#include "core/job_system/job.h"
#include "core/job_system/fiber.h"
#include "engine/engine.h"
#include "ply.h"
#include "graphics/rhi/window.h"
#include "graphics/rhi/rhi.h"
#include "graphics/rhi/frame_buffer.h"
#include "graphics/rhi/pipeline.h"
#include "graphics/assets/assets.h"

#include "graphics/renderer/renderer.h"
#include "ecs/ecs.h"
#include "components/camera.h"
#include "components/transform.h"
#include "components/flyover.h"
#include "components/lights.h"

#include "core/context.h"

#include "point_cloud.h"
#include "ui/ui.h"

#include "graphics/rhi/vulkan/vulkan.h"
#include "graphics/rhi/vulkan/buffer.h"
#include "graphics/rhi/vulkan/draw.h"

#include <iostream>
#include <queue>

using namespace std::chrono;

using namespace eth_localization;

void convert() {
    Convert_Point_Cloud_Options options = {};
    options.max_verts_in_memory = mb(10);

    const char* src = "pointcloud_hg.ply";
    const char* dst = "pointcloud_hg.pc";

    AABB aabb = {{-80.2021,-160.143,-14.5384}, {173.286,107.418,46.9525}};

    eth_localization::convert_point_cloud(dst, src, aabb, options);
}

void init_workers(void*) {
    printf("Initialized worker %i\n", get_worker_id());
    get_thread_local_permanent_allocator() = LinearAllocator(mb(10));
    get_thread_local_temporary_allocator() = LinearAllocator(mb(100));

    Context& ctx = get_context();
    ctx.allocator = &default_allocator;
    ctx.temporary_allocator = &get_thread_local_temporary_allocator();
}

// class RenderSystem {
//public:
//    virtual void extract_render_data(RenderPrepareCtx& ctx) = 0;
//    virtual void build_command_buffers(RenderSubmitCtx& ctx) = 0;
//    virtual ~RenderSystem();
//};
class Point_Class_Render_Module : public RenderSystem {
    model_handle model;
    VertexBuffer vertex_buffer;
    shader_handle shader;
    pipeline_handle pipeline;
    LayoutVertexInputs vertex_layouts;
    StagingQueue& staging;
    VertexStreaming vertex_streaming;

    uint32_t waiting_on_transfer_frame = 0;

    u64 page_size = 16*1024*sizeof(PLY_Vertex);
    std::vector<u64> free_pages;

    struct Node_Info {
        enum class Commitment {
            NOT_LOADED,
            GPU_MEMORY,
        } commitment = Commitment::NOT_LOADED;
        enum Stage {
            Loading,
            Uploading,
            Rendering,
            Deallocating,
        } stage = Stage::Loading;
        u64 vert_buffer = 0;
    };

    u64 root;
    std::vector<PPC_Node> nodes;
    std::vector<Node_Info> node_info;
    PPCloud cloud;

    std::vector<u64> load_nodes;

    float alpha = 2;
    float stream_dist_threshold = 6000;
    float view_dist_threshold = 6000;
    Renderer& renderer;

    struct dist_node {
        float dist = 0;
        u64 id = 0;
        u64 depth = 0;

        bool operator<(dist_node node) const {
            return id > node.id;
        }
    };

    std::priority_queue<dist_node> node_queue;
public:
    Point_Class_Render_Module(const char* file, Renderer& renderer)
    : staging(rhi.staging_queue),
    vertex_streaming{rhi.staging_queue},
    cloud(file, PPCloud::in), renderer(renderer) {
        PPC_Node node = cloud.read_node(cloud.root());

        shader = load_Shader("point_shader.vert", "point_shader.frag");

        VertexLayout layout;
        {
            VertexLayoutDesc desc = {
                {
                        {3,VertexAttrib::Type::Float, offsetof(PLY_Vertex, pos)},
                        {3,VertexAttrib::Type::Float, offsetof(PLY_Vertex,normal)},
                        {3,VertexAttrib::Type::Unorm, offsetof(PLY_Vertex,color)},
                },
                sizeof(PLY_Vertex),
            };
            layout = register_vertex_layout(desc);
            fill_layout(vertex_layouts, VK_VERTEX_INPUT_RATE_VERTEX, desc.attribs, desc.elem_size, 0);
        }

        nodes = cloud.read_nodes(0, cloud.node_count()+1);
        node_info.resize(nodes.size()+1);
        root = cloud.root();

        Device& device = rhi.device;
        VkPhysicalDevice physical_device = device.physical_device;

        uint64_t vertex_max_memory = gb(2.5);
        uint64_t index_max_memory = mb(128);
        uint64_t staging_memory = gb(2);
        make_VertexStreaming(vertex_streaming, device.device, device.physical_device, staging,
                             {&vertex_layouts,1},
                             &vertex_max_memory,&index_max_memory, staging_memory);

        for(uint64_t i = 0; i+page_size < vertex_max_memory; i += page_size) {
            free_pages.push_back(i);
        }

        {
            GraphicsPipelineDesc desc = {};
            desc.render_pass = RenderPass::Scene;
            desc.vertex_layout = layout;
            desc.instance_layout = INSTANCE_LAYOUT_NONE;
            desc.shader = shader;
            desc.state = PrimitiveType_PointList;

            desc.range[0].size = sizeof(uint32_t);
            desc.range[0].offset = 0;

            pipeline = query_Pipeline(desc);
        }
    }

    void alloc_node(PPC_Node& node, Node_Info& info) {
        if(free_pages.empty()) {
            std::cerr << "Out of free pages" << std::endl;
            return;
        }

        u64 page = free_pages.back();
        free_pages.pop_back();

        printf("LOADING NODE %ul, with %ul vertices\n", node.id, node.vert_count);

        std::vector<PLY_Vertex> verts = cloud.read_verts(node);
        assert(verts.size()*sizeof(PLY_Vertex) <= page_size);
        staged_copy(vertex_streaming, vertex_streaming.vertex_buffer, page, verts.data(), verts.size()*sizeof(PLY_Vertex));

        info.commitment = Node_Info::Commitment::GPU_MEMORY;
        info.vert_buffer = page;
    }

    void dealloc_node(PPC_Node& node, Node_Info& info) {
        printf("DEALLOCATING NODE %ul\n", node.id);

        free_pages.push_back(info.vert_buffer);
        info.commitment = Node_Info::Commitment::NOT_LOADED;
    }

    struct FrameData {
        glm::vec3 viewer;
    } frames[MAX_FRAMES_IN_FLIGHT];

    Node_Info& get_info(u64 id) {
        return node_info[id];
    }

    float compute_dist(PPC_Node node, glm::vec3 viewer) {
        glm::vec3 viewer_inside;
        viewer_inside.x = std::clamp(viewer.x, node.aabb.min.x, node.aabb.max.x);
        viewer_inside.y = std::clamp(viewer.y, node.aabb.min.z, node.aabb.max.z);
        viewer_inside.z = std::clamp(viewer.z, node.aabb.min.y, node.aabb.max.y);

        float dist = glm::length(viewer - viewer_inside);
        return dist;
    }

    void update_nodes(glm::vec3 viewer) {
        node_queue.push({compute_dist(nodes[cloud.root()], viewer), cloud.root(), 0});

        while(!node_queue.empty()) {
            auto[dist,node_id,depth] = node_queue.top();
            PPC_Node node = nodes[node_id];
            node_queue.pop();

            bool keep_node = dist * (pow(alpha, depth)) < view_dist_threshold;
            Node_Info &info = get_info(node.id);

            Node_Info::Commitment prev_commitment = info.commitment;
            switch (info.commitment) {
                case Node_Info::Commitment::NOT_LOADED:
                    if (keep_node) alloc_node(node, info);
                    break;
                case Node_Info::Commitment::GPU_MEMORY:
                    if (!keep_node) dealloc_node(node, info);
                    break;
            };
            bool commitment_changed = info.commitment != prev_commitment;
            if (info.commitment != Node_Info::Commitment::NOT_LOADED || commitment_changed) {
                for (uint32_t i = 0; i < PPC_Node::CHILDREN; i++) {
                    if (node.children[i] != 0) {
                        node_queue.push({compute_dist(nodes[node.children[i]], viewer), node.children[i], depth+1});
                    }
                }
            }
        }
    }

    virtual void extract_render_data(RenderPrepareCtx& ctx) {
        World& world = ctx.get_world();
        auto[e,trans,camera] = *world.first<Transform,Camera>(ctx.get_camera_layermask());

        frames[ctx.get_frame()].viewer = trans.position;
        begin_vertex_buffer_upload(vertex_streaming);
        update_nodes(trans.position);
        end_vertex_buffer_upload(vertex_streaming);
    }

    bool build_command_buffers(CommandBuffer& cmd_buffer, FrameData& frame, uint32_t depth, PPC_Node& node) {
        Node_Info& info = get_info(node.id);
        if(info.commitment == Node_Info::Commitment::NOT_LOADED) return false;

        float dist = compute_dist(node, frame.viewer);

        bool draw = dist*pow(alpha,depth) <= view_dist_threshold;

        if(!draw) return false;

        bool child_rendered = true;
        for(uint64_t i = 0; i < PPC_Node::CHILDREN; i++) {
            if(node.children[i] != 0) {
                child_rendered = child_rendered && build_command_buffers(cmd_buffer, frame, depth + 1, nodes[node.children[i]]);
            } else {
                child_rendered = false;
            }
        }
        if(child_rendered) return true;
        //if(node.has_children && dist*(1<<(1+depth)) <= view_dist_threshold) return false;

        glm::vec3 size3 = node.aabb.size();
        float size = glm::length(glm::vec2(size3.x,size3.y));
        size = std::min(size, 5.f);
        //size = size*size;
        push_constant(cmd_buffer, Stage::VERTEX_STAGE, 0, &size);
        vkCmdDraw(cmd_buffer.cmd_buffer, node.vert_count, 1, info.vert_buffer/sizeof(PLY_Vertex), 0);
        return true;
    }

    virtual void build_command_buffers(RenderSubmitCtx& ctx) {
        RenderPass pass = ctx.get_render_pass(RenderPass::Scene);
        CommandBuffer& cmd_buffer = *pass.cmd_buffer;
        bind_pipeline(cmd_buffer, pipeline);

        bind_descriptor(cmd_buffer, 0, renderer.scene_pass_descriptor[ctx.get_frame()]);

        uint64_t offset = 0;
        vkCmdBindVertexBuffers(cmd_buffer.cmd_buffer, 0, 1, &vertex_streaming.vertex_buffer, &offset);

        build_command_buffers(cmd_buffer, frames[ctx.get_frame()], 0, nodes[root]);
    }
};


void set_theme(UITheme& theme) {
    theme
            .color(ThemeColor::Text, white)
            .color(ThemeColor::Button, shade3)
            .color(ThemeColor::ButtonHover, shade3)
            .color(ThemeColor::Panel, shade1)
            .color(ThemeColor::Splitter, shade1)
            .color(ThemeColor::SplitterHover, blue)
            .color(ThemeColor::Input, shade1)
            .color(ThemeColor::InputHover, shade3)
            .color(ThemeColor::Cursor, blue)
            .size(ThemeSize::DefaultFontSize, 10)
            .size(ThemeSize::Title1FontSize, 13)
            .size(ThemeSize::Title2FontSize, 12)
            .size(ThemeSize::Title3FontSize, 11)
            .size(ThemeSize::SplitterThickness, 1)
            .size(ThemeSize::VStackMargin, 0)
            .size(ThemeSize::HStackMargin, 0)
            .size(ThemeSize::VStackPadding, 4)
            .size(ThemeSize::HStackPadding, 0)
            .size(ThemeSize::StackSpacing, 4)
            .size(ThemeSize::VecSpacing, 2)
            .size(ThemeSize::TextPadding, 0)
            .size(ThemeSize::InputPadding, 4)
            .size(ThemeSize::InputMinWidth, 60)
            .size(ThemeSize::InputMaxWidth, {Perc, 20});
}

int main() {
    try {
        const char *point_cloud_filename = "pointcloud.pc";
        ////convert();
        //return 0;

        make_job_system(512, 1);

        JobDesc init_jobs[MAX_THREADS];
        uint init_jobs_on[MAX_THREADS];

        uint32_t num_workers = 1;
        for (int i = 0; i < num_workers - 1; i++) {
            init_jobs[i] = JobDesc{init_workers, nullptr};
            init_jobs_on[i] = i + 1;
        }

        atomic_counter counter = 0;
        schedule_jobs_on({init_jobs_on, num_workers - 1}, {init_jobs, num_workers - 1}, &counter);

        get_thread_local_temporary_allocator() = LinearAllocator(mb(32));
        get_thread_local_permanent_allocator() = LinearAllocator(kb(16));

        get_context().temporary_allocator = &get_thread_local_temporary_allocator();

        Modules modules("viz", "", "../viz/vendor/NextEngine/NextEngine/data/");

        modules.init_graphics();

        convert_thread_to_fiber();

        Dependency dependencies[2] = {
                {FRAGMENT_STAGE, RenderPass::Composite, TextureAspect::Color},
                {FRAGMENT_STAGE, RenderPass::Scene,     TextureAspect::Color | TextureAspect::Depth},
        };
        make_wsi_pass({dependencies, 2});
        build_framegraph();

        UI *ui = make_ui(*modules.renderer);
        set_theme(get_ui_theme(*ui));
        load_font(*ui, "segoeui.ttf");

        modules.renderer->add(std::make_unique<Point_Class_Render_Module>(point_cloud_filename, *modules.renderer));
        end_gpu_upload();

        World &world = *modules.world;
        {
            auto [entity, camera, trans, flyover] = world.make<Camera, Transform, Flyover>();
            camera.near_plane = 0.1;
            camera.far_plane = 200;
            trans.position = {0, 0, 5};
            flyover.mouse_sensitivity = 2;
        }

        {
            auto [entity, trans, dir_light] = world.make<Transform, DirLight>();
            trans.position = vec3(10, 10, 0);
            dir_light.direction = normalize(vec3(-1, -1, -1));
        }

        /*{
            auto [entity,trans,model,materials] = world.make<Transform,ModelRenderer,Materials>();
            model.model_id = primitives.sphere;
            trans.position = {0,0,0};
            trans.scale = {1,1,1};
            materials.materials.append(default_materials.missing);
        }*/

        while (!modules.window->should_close()) {
            get_temporary_allocator().clear();
            modules.begin_frame();

            World &world = *modules.world;
            UpdateCtx ctx(*modules.time, *modules.input);

            update_flyover(world, ctx);
            update_local_transforms(world, ctx);

            ScreenInfo info = modules.window->get_screen_info();
            Viewport viewport = {};
            viewport.width = info.fb_width;
            viewport.height = info.fb_height;

            begin_ui_frame(*ui, info, *modules.input, CursorShape::Arrow);
            begin_vstack(*ui).width({Perc, 100}).height({Perc, 100}).background({0.5, 0.5, 0.5, 1.0});
            text(*ui, "3D Vision - ETH Localization").color({0.0, 0.0, 0.0, 1.0}).background({1.0, 1.0, 1.0, 1.0});
            image(*ui, modules.renderer->get_output_map()).resizeable();
            end_vstack(*ui);
            end_ui_frame(*ui);

            begin_gpu_upload();
            RenderPrepareCtx prepare = modules.renderer->extract_render_data(*modules.world, viewport, EntityQuery(),
                                                                             EntityQuery());
            end_gpu_upload();

            RenderSubmitCtx submit = modules.renderer->build_command_buffers(prepare);
            modules.renderer->end_passes(submit);
            modules.renderer->submit_frame(submit);

            modules.end_frame();
        }

        destroy_job_system();
    } catch(std::string& e) {
        std::cerr << "\nINTERNAL ERROR " << e << std::endl;
    } catch (string_buffer& e) {
        std::cerr << "\nINTERNAL ERROR " << e.c_str() << std::endl;
    }
    catch (std::invalid_argument& e) {
        std::cerr << "\nINTERNAL ERROR " << e.what() << std::endl;
    }
    catch(const char* e) {
        std::cerr << "\nINTERNAL ERROR " << e << std::endl;
    }
}