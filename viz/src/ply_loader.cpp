#include <ostream>
#include <fstream>
#include <sstream>
#include <iostream>
#include "ply.h"
#include "mpc_queue.h"
#include "logging.h"
#include "point_cloud.h"
#include "core/core.h"

namespace eth_localization {
    struct Prop {
        enum class ID {
            None,
            X,
            Y,
            Z,
            R,
            G,
            B,
            NX,
            NY,
            NZ,
            Count
        } id;
        enum class Type {
            None,
            Float,
            UChar,
            Int,
            Count
        } type;

        static constexpr uint32_t PROP_SIZE[(int)Type::Count] = {0,4,1,4};
    };

    struct PLY_Header {
        size_t num_cols = 0;
        std::vector<Prop> props;
    };

    std::ostream& operator<<(std::ostream &o, const PLY_Header& header) {
        o << "=== PLY HEADER ===" << std::endl;
        o << "num-cols -" << header.num_cols << std::endl;
        return o;
    }

    PLY_Header parse_header(std::ifstream &fin) {
        std::string line;

        PLY_Header header;

        bool first = false;
        while (std::getline(fin, line)) {
            std::istringstream words(line);
            std::string kind;
            words >> kind;

            if (first && kind != "ply") throw std::invalid_argument("Expecting ply format");
            else if (kind == "format") {
                std::string format, version;
                words >> format >> version;
                if (!(format == "binary_little_endian" && version == "1.0")) {
                    throw std::invalid_argument("Expecting binary_little endian version 1");
                }
            } else if (kind == "comment") {}
            else if (kind == "obj_info") {
                std::string property;
                words >> property;

                if (property == "num_cols") {
                    words >> header.num_cols;
                }
            }
            else if (kind == "element") {
                std::string type;
                words >> type;
                if(type == "vertex") {
                    words >> header.num_cols;
                }
            }
            else if (kind == "property") {
                std::string id, type;
                words >> type >> id;

                static std::unordered_map<std::string,Prop::ID> map_id = {
                        {"x", Prop::ID::X},
                        {"y", Prop::ID::Y},
                        {"z", Prop::ID::Z},
                        {"red", Prop::ID::R},
                        {"green", Prop::ID::G},
                        {"blue", Prop::ID::B},
                        {"nx", Prop::ID::NX},
                        {"ny", Prop::ID::NY},
                        {"nz", Prop::ID::NZ}
                };
                static std::unordered_map<std::string,Prop::Type> map_type = {
                        {"float", Prop::Type::Float},
                        {"uchar", Prop::Type::UChar},
                        {"char", Prop::Type::UChar},
                        {"int", Prop::Type::Int},
                };

                Prop prop = {};
                prop.id = map_id[id];
                prop.type = map_type[type];

                if(prop.id == Prop::ID::None) {
                    std::cerr << "Unknown property " << id << std::endl;
                }

                if(prop.type == Prop::Type::None) {
                    throw std::invalid_argument("Unknown type " + type + " " + id);
                }

                header.props.push_back(prop);
            }
            else if (kind == "end_header") break;

            first = false;
        }

        std::cout << header;

        return header;
    }

    std::vector<PLY_Vertex> parse_vertices(std::ifstream& fin,
                                           const PLY_Header& header,
                                           uint32_t chunk_size, uint32_t max_chunks) {
        std::vector<PLY_Vertex> result;
        Prop::Type prop_types[(int)Prop::ID::Count] = {};
        uint32_t prop_offsets[(int)Prop::ID::Count] = {};

        uint32_t prop_size = 0;
        for(Prop prop : header.props) {
            prop_types[(int)prop.id] = prop.type;
            prop_offsets[(int)prop.id] = prop_size;
            prop_size += Prop::PROP_SIZE[(int)prop.type];
        }

        auto has_vec = [&](Prop::ID x, Prop::ID y, Prop::ID z, Prop::Type type) {
            return prop_types[(int)x] == type && prop_types[(int)y] == type && prop_types[(int)z] == type;
        };

        bool has_pos = has_vec(Prop::ID::X,Prop::ID::Y,Prop::ID::Z,Prop::Type::Float);
        bool has_color = has_vec(Prop::ID::R,Prop::ID::G,Prop::ID::B,Prop::Type::UChar);
        bool has_normal = has_vec(Prop::ID::NX,Prop::ID::NY,Prop::ID::NZ, Prop::Type::Float);
        if(!has_pos) {
            throw std::invalid_argument("Expecting pos and color");
        }

        std::vector<char> buffer(chunk_size * prop_size, 0);
        for(uint32_t chunks = 0; !fin.eof() && chunks < max_chunks; chunks++) {
            fin.read(buffer.data(), buffer.size());

            std::size_t size = fin.gcount() / prop_size;

            auto parse_float = [&](char* data) {
                return *((float*)data); // todo: flip bytes on big-endian machine
            };

            auto parse_byte = [&](char* data) {
                return *data; // todo: flip bytes on big-endian machine
            };

            for(int i = 0; i < size; i++) {
                size_t offset = i*prop_size;

                PLY_Vertex vertex = {};

                if(has_pos) {
                    vertex.pos.x = parse_float(buffer.data() + offset + prop_offsets[(int)Prop::ID::X]);
                    vertex.pos.y = parse_float(buffer.data() + offset + prop_offsets[(int)Prop::ID::Y]);
                    vertex.pos.z = parse_float(buffer.data() + offset + prop_offsets[(int)Prop::ID::Z]);
                }

                if(has_color) {
                    vertex.color.x = parse_byte(buffer.data() + offset + prop_offsets[(int)Prop::ID::R]);
                    vertex.color.y = parse_byte(buffer.data() + offset + prop_offsets[(int)Prop::ID::G]);
                    vertex.color.z = parse_byte(buffer.data() + offset + prop_offsets[(int)Prop::ID::B]);
                }

                if(has_normal) {
                    vertex.normal.x = parse_float(buffer.data() + offset + prop_offsets[(int)Prop::ID::NX]);
                    vertex.normal.y = parse_float(buffer.data() + offset + prop_offsets[(int)Prop::ID::NY]);
                    vertex.normal.z = parse_float(buffer.data() + offset + prop_offsets[(int)Prop::ID::NZ]);
                }

                result.push_back(vertex);
            }
        }

        return result;
    }

    struct PLY_Chunk {
        uint32_t id;
        std::vector<PLY_Vertex> vertices;
    };

    constexpr uint32_t CHILDREN = PPC_Node::CHILDREN;

    uint32_t quadrant_index(AABB aabb, glm::vec3 pos) {
        glm::vec3 offset = 2.f*(pos - aabb.min) / aabb.size();
        int index = std::clamp((int)offset.x,0,1) + 2 * std::clamp((int)offset.y,0,1); // + 4*std::clamp((int)offset.z,0,1);
        return index;
    }

    void partition_node_in_memory(uint32_t node_id, std::vector<PLY_Vertex>& verts, std::vector<PLY_Vertex>& my_verts,std::vector<PPC_Node>& nodes,
                        const Convert_Point_Cloud_Options& options) {
        uint64_t vert_count = nodes[node_id].vert_count;
        if(vert_count <= options.verts_per_node) {
            u64 offset = nodes[node_id].vert_offset;
            for (uint64_t j = 0; j < my_verts.size(); j++) verts[offset + j] = my_verts[j];
            for(int i = 0; i < CHILDREN; i++) nodes[node_id].children[i] = -1;
            return;
        }

        AABB aabb = nodes[node_id].aabb;
        std::vector<PLY_Vertex> child_verts[CHILDREN];
        AABB child_aabbs[CHILDREN];
        uint64_t vert_offset = nodes[node_id].vert_offset;

        for(uint64_t i = 0; i < vert_count; i++) {
            PLY_Vertex& vertex = my_verts[i];
            uint32_t index = quadrant_index(aabb, vertex.pos);
            child_verts[index].push_back(vertex);
            child_aabbs[index].update(vertex.pos);
        }

        uint64_t offset = nodes[node_id].vert_offset;
        nodes[node_id].has_children = true;

        std::vector<PLY_Vertex> downsampled_verts;
        offset = nodes[node_id].vert_offset;
        for(uint32_t i = 0; i < CHILDREN; i++) {
            if(child_verts[i].empty()) nodes[node_id].children[i] = -1;
            else {
                uint64_t child_id = nodes.size();
                nodes[node_id].children[i] = child_id;
                nodes.push_back({});
                nodes[child_id].id = child_id;
                nodes[child_id].vert_count = child_verts[i].size();
                nodes[child_id].aabb = child_aabbs[i];
                nodes[child_id].vert_offset = offset;
                partition_node_in_memory(child_id, verts, child_verts[i], nodes, options);

                for(uint64_t j = 0; j < nodes[child_id].vert_count; j += CHILDREN) {
                    downsampled_verts.push_back(verts[nodes[child_id].vert_offset+j]);
                }
                offset += child_verts[i].size();
            }
        }

        nodes[node_id].vert_offset = verts.size();
        nodes[node_id].vert_count = downsampled_verts.size();
        verts.insert(verts.end(), downsampled_verts.begin(), downsampled_verts.end());
    }

    void partition_node(PPCloud& cloud, PPC_Node& node, Progess_Log& log, const Convert_Point_Cloud_Options& options) {
        if(node.vert_count < options.verts_per_node) return;

        log.incr_target(1);

        if(node.vert_count <= options.max_verts_in_memory) {
            std::vector<PLY_Vertex> verts = cloud.read_verts(node);

            std::vector<PPC_Node> nodes = {node};
            nodes[0].id = 0;
            nodes[0].vert_offset = 0;
            partition_node_in_memory(0, verts, verts, nodes, options);

            uint64_t vert_count = 0;
            for(uint32_t i = 0; i < nodes.size(); i++) {
                if(nodes[i].has_children) vert_count += nodes[i].vert_count; // leaf node data already allocated
            }

            uint64_t node_base = cloud.alloc_nodes(nodes.size()-1);
            uint64_t vert_base = cloud.alloc_verts(vert_count);
            for(uint64_t i = 0; i < nodes.size(); i++) {
                PPC_Node& child = nodes[i];
                child.id = i==0 ? node.id : node_base+child.id-1;
                if(child.has_children) {
                    child.vert_offset = vert_base + child.vert_offset - node.vert_count;
                    for (uint32_t j = 0; j < CHILDREN; j++) {
                        if (child.children[j] == -1) child.children[j] = 0;
                        else child.children[j] = node_base+child.children[j]-1;
                    }
                } else {
                    child.vert_offset += node.vert_offset;
                    for (uint32_t j = 0; j < CHILDREN; j++) child.children[j] = 0;
                }
            }

            // todo: assumes linear allocation
            cloud.write_nodes(node.id, {nodes[0]});
            cloud.write_nodes(node_base,{nodes.begin()+1,nodes.end()});
            cloud.write_verts(node.vert_offset, {verts.begin(), verts.begin()+node.vert_count});
            cloud.write_verts(vert_base,{verts.begin()+node.vert_count, verts.end()});

            node = nodes[0];
        } else {
            AABB child_aabbs[CHILDREN];
            uint64_t child_count[CHILDREN] = {};

            auto for_each_vert = [&](auto&& fn) {
                for(uint64_t vert = 0; vert < node.vert_count; vert += options.read_buffer) {
                    std::vector<PLY_Vertex> verts = cloud.read_verts(node.vert_offset + vert, std::min(options.read_buffer, node.vert_count - vert));
                    for (uint64_t i = 0; i < verts.size(); i++) {
                        fn(quadrant_index(node.aabb, verts[i].pos), verts[i]);
                    }
                }
            };

            for_each_vert([&](int index, PLY_Vertex vertex) {
                child_aabbs[index].update(vertex.pos);
                child_count[index]++;
            });

            std::vector<PPC_Node> child_nodes = cloud.alloc_nodes_with({0,0,0,0});
            uint64_t vert_offsets[CHILDREN];
            uint64_t offset = node.vert_offset;
            for(uint32_t j = 0; j < CHILDREN; j++) {
                node.children[j] = child_nodes[j].id;
                child_nodes[j].aabb = child_aabbs[j];
                child_nodes[j].vert_count = child_count[j];
                child_nodes[j].vert_offset = offset;
                vert_offsets[j] = offset;
                offset += child_count[j];
            }

            std::vector<PLY_Vertex> assigned_verts[CHILDREN];

            for_each_vert([&](int quadrant, PLY_Vertex vertex) {
                std::vector<PLY_Vertex>& stream = assigned_verts[quadrant];
                stream.push_back(vertex);
                if(stream.size() >= options.write_buffer) {
                    cloud.write_verts(vert_offsets[quadrant], stream);
                    vert_offsets[quadrant] += stream.size();
                    stream.clear();
                }
            });
            for(uint32_t quadrant = 0; quadrant < CHILDREN; quadrant++) {
                cloud.write_verts(vert_offsets[quadrant], assigned_verts[quadrant]);
            }

            cloud.write_nodes(node.id, {node});
            cloud.write_nodes(child_nodes[0].id, child_nodes);

            uint64_t vert_count = 0;
            for(uint32_t quadrant = 0; quadrant < CHILDREN; quadrant++) {
                partition_node(cloud, child_nodes[quadrant], log, options);
                vert_count += divceil(child_nodes[quadrant].vert_count, CHILDREN);
            }

            uint64_t vert_base = cloud.alloc_verts(vert_count);
            uint64_t vert_offset = vert_base;

            std::vector<PLY_Vertex> downsampled_verts;
            for(uint32_t quadrant = 0; quadrant < CHILDREN; quadrant++) {
                uint64_t child_vert_count = child_nodes[quadrant].vert_count;
                std::vector<PLY_Vertex> verts = cloud.read_verts(child_nodes[quadrant]); // top-level has few verts,in-memory okay
                for(uint32_t j = 0; j < child_vert_count; j += CHILDREN) {
                    downsampled_verts.push_back(verts[j]);
                }
            }
            assert(downsampled_verts.size() <= vert_count);
            node.vert_offset = vert_base;
            node.vert_count = downsampled_verts.size();
            cloud.write_nodes(node.id, {node});
            cloud.write_verts(node.vert_offset, downsampled_verts);
        }

        log.update();
    }

    void partition_into_quadtree(PPCloud& cloud, const Convert_Point_Cloud_Options& options) {
        Progess_Log log("Building quadtree", 0);
        partition_node(cloud, cloud.read_nodes(cloud.root(),1)[0], log, options);
    }

    void convert_point_cloud(const std::string &dst,
                             const std::string& src,
                             AABB aabb,
                             const Convert_Point_Cloud_Options& options) {
        const char *MARKER = "end_header";

        std::ifstream fin(src, std::ifstream::binary);
        PLY_Header header = parse_header(fin);

        uint64_t vert_count = header.num_cols;
        uint64_t vert_capacity = 1.5*header.num_cols;
        uint64_t node_capacity = 20*header.num_cols / options.verts_per_node; // todo: compute better upper bound

        PPCloud pcloud(dst, PPCloud::Mode(PPCloud::in | PPCloud::out | PPCloud::create));
        pcloud.reserve(node_capacity, vert_capacity);

        Progess_Log progess("Loading ply", std::filesystem::file_size(src));

        PPC_Node node = pcloud.alloc_nodes_with({vert_count})[0];
        node.aabb = aabb;
        pcloud.write_nodes(node.id, {node});
        pcloud.set_root(node.id);
        node.vert_count = 0;
        std::vector<PLY_Vertex> vertices;
        for(uint32_t i = 0; !fin.eof(); i++) {
            vertices = parse_vertices(fin, header, options.read_buffer, 1);
            pcloud.write_verts(node.vert_offset + node.vert_count, vertices);
            node.vert_count += vertices.size();
            progess.update(fin.tellg());
        }
        pcloud.write_nodes(node.id, {node});
        std::cout << "Point cloud has " << node.vert_count << "vertices" << std::endl;

        partition_into_quadtree(pcloud, options);
    }
}