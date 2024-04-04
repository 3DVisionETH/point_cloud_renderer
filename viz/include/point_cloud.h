#include <vector>
#include <glm/vec3.hpp>
#include <core/math/aabb.h>
#include <fstream>
#include "ply.h"
#include <span>

namespace eth_localization {
    // Persistent Point Cloud Node
    struct PPC_Node {
        static constexpr uint32_t CHILDREN = 4;

        uint64_t id = {};
        enum Mode {
            XY,
            XZ,
            YZ
        } mode : 4;
        bool has_children : 1 = {};
        AABB aabb;
        uint64_t vert_count = {};
        uint64_t vert_offset = {};
        uint64_t children[CHILDREN] = {};
    };

    // Persistent Point Cloud
    class PPCloud {
    public:
        enum Mode {
            in = 1,
            out = 2,
            create = 4
        };
    private:
        struct Header {
            uint64_t file_size = 0;
            uint64_t vertices_section_offset = 0;
            uint64_t node_section_offset = 0;
            uint64_t vertex_count = 0;
            uint64_t vertex_capacity = 0;
            uint64_t node_count = 0;
            uint64_t node_capacity = 0;
            uint64_t root = 0;
        };

        Mode mode;
        bool readonly = false;

        std::string filename;
        std::fstream file;

        Header header;

        uint64_t header_section() const;

        uint64_t vertices_section() const;

        uint64_t nodes_section() const;

        void create_new();

        std::ios::openmode file_openmode();
    public:
        PPCloud(const std::string& filename, Mode mode);
        ~PPCloud();

        void reserve(uint64_t nodes, uint64_t verts);

        uint64_t alloc_nodes(uint64_t count);
        std::vector<PPC_Node> alloc_nodes_with(const std::vector<uint64_t>& vertex_per_node);
        uint64_t alloc_verts(uint64_t count);

        uint64_t node_count() const;
        uint64_t vertex_count() const;

        uint64_t root();
        void set_root(uint64_t id);

        void read_header();

        void write_header();

        void write_nodes(uint64_t base, const std::vector<PPC_Node>& nodes);
        void write_nodes(const std::vector<PPC_Node>& nodes);

        void write_verts(uint64_t base, const std::vector<PLY_Vertex>& verts);

        void write_verts(const PPC_Node& node, const std::vector<PLY_Vertex>& verts);

        void flush();

        PPC_Node read_node(uint64_t id);

        std::vector<PPC_Node> read_nodes(uint64_t base, uint64_t count);

        std::vector<PLY_Vertex> read_verts(uint64_t base, uint64_t count);

        std::vector<PLY_Vertex> read_verts(const PPC_Node& node);
    };
}