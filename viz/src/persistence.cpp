#include "point_cloud.h"
#include <strstream>
#include <iostream>

namespace eth_localization {
    uint64_t PPCloud::header_section() const {
        return 0;
    }

    uint64_t PPCloud::vertices_section() const {
        return header.vertices_section_offset;
    }

    uint64_t PPCloud::nodes_section() const {
        return header.node_section_offset;
    }

    uint64_t PPCloud::node_count() const {
        return header.node_count;
    }

    uint64_t PPCloud::vertex_count() const {
        return header.vertex_count;
    }

    uint64_t PPCloud::root() { return header.root; }

    void PPCloud::set_root(uint64_t id) {
        header.root = id;
        write_header();
    }

    std::ios::openmode PPCloud::file_openmode() {
        std::ios::openmode result = std::ios::binary;
        if(mode & in) result |= std::ios::in;
        if(mode & out) result |= std::ios::out;
        return result;
    }

    PPCloud::PPCloud(const std::string &filename, Mode mode)
    : mode(mode), filename(filename) {
        if(mode & Mode::create) {
            std::fstream f(filename, std::ios::binary | std::ios::out); // Create empty file
        }

        file = std::fstream(filename, file_openmode());
        if(!file.good()) throw std::invalid_argument("Cannot create ppcloud : " + std::to_string(errno));

        readonly = !(mode & out);

        if(mode == out || mode & create) {
            create_new();
        } else {
            read_header();
        }
    }

    PPCloud::~PPCloud() {
        if(!readonly) flush();
    }

    void PPCloud::reserve(uint64_t nodes, uint64_t verts) {
        assert(!readonly);

        if(header.file_size != 0) {
            std::cerr << "Resizing non-empty file, data may be lost" << std::endl;
        }

        constexpr uint64_t block_alignment = 1024;
        static_assert(sizeof(Header) < block_alignment);

        auto align = [](uint64_t offset, uint64_t align) {
            if(offset % align == 0) return offset;
            return offset / align * align + align;
        };

        header.node_section_offset = block_alignment;
        header.vertices_section_offset = align(block_alignment + sizeof(PPC_Node) * nodes, block_alignment);
        header.file_size = align(header.vertices_section_offset + sizeof(PLY_Vertex)*verts, block_alignment);

        header.node_capacity = nodes;
        header.vertex_capacity = verts;

        if(!file.is_open()) {
            file.close();
            file = std::fstream(filename, std::ios::out | std::ios::binary);
            file.close();
        } else {
            file.close();
        }
        std::filesystem::resize_file(filename, header.file_size);

        file = std::fstream(filename, std::ios::in | std::ios::out | std::ios::binary);

        write_header();
    }

    uint64_t PPCloud::alloc_verts(uint64_t vertex_count) {
        if(header.vertex_count + vertex_count >= header.vertex_capacity) {
            std::strstream s;
            s << "Out of vertices : [" << header.vertex_count << "/" << header.vertex_capacity << " - " << vertex_count;
            throw std::invalid_argument(s.str());
        }
        uint64_t vert_base = header.vertex_count+1;
        header.vertex_count += vertex_count;
        return vert_base;
    }

    uint64_t PPCloud::alloc_nodes(uint64_t node_count) {
        if(header.node_count + node_count >= header.node_capacity) {
            std::strstream s;
            s << "Out of nodes : [" << header.node_count << "/" << header.node_capacity << " - " << node_count;
            throw std::invalid_argument(s.str());
        }
        uint64_t node_base = header.node_count+1;
        header.node_count += node_count;
        return node_base;
    }

    std::vector<PPC_Node> PPCloud::alloc_nodes_with(const std::vector<uint64_t>& vertex_per_node) {
        size_t node_count = vertex_per_node.size();
        uint64_t vertex_count = 0;
        for(size_t x : vertex_per_node) vertex_count += x;

        uint64_t node_base = alloc_nodes(node_count);
        uint64_t vert_base = alloc_verts(vertex_count);

        std::vector<PPC_Node> nodes(node_count);

        for(size_t i = 0; i < vertex_per_node.size(); i++) {
            nodes[i].id = node_base;
            nodes[i].vert_offset = vert_base;
            nodes[i].vert_count = vertex_per_node[i];
            vert_base += vertex_per_node[i];
            node_base++;
        }

        return nodes;
    }

    void PPCloud::create_new() {
        write_header();
    }

    void PPCloud::write_header() {
        assert(!readonly);
        file.seekp(header_section());
        file.write(reinterpret_cast<const char*>(&header), sizeof(Header));
    }

    void PPCloud::write_nodes(uint64_t base, const std::vector<PPC_Node> &nodes) {
        assert(!readonly);
        file.seekp(nodes_section() + sizeof(PPC_Node) * base);
        file.write(reinterpret_cast<const char*>(nodes.data()), nodes.size()*sizeof(PPC_Node));
    }

    void PPCloud::write_nodes(const std::vector<PPC_Node> &nodes) {
        if(nodes.empty()) return;

        size_t start_group = 0;
        for(size_t i = 1; i < nodes.size(); i++) {
            bool is_monotonic = nodes[i-1].id+1 == nodes[i].id;
            if(!is_monotonic) {
                write_nodes(start_group, {nodes.begin()+start_group, nodes.begin()+i}); // todo: use std::span
                start_group = i;
            }
        }
        if(start_group == 0) write_nodes(nodes[0].id, nodes);
        else write_nodes(start_group, {nodes.begin()+start_group, nodes.end()});
    }

    void PPCloud::write_verts(uint64_t base, const std::vector<PLY_Vertex> &verts) {
        assert(!readonly);
        file.seekp(vertices_section() + sizeof(PLY_Vertex)*base);
        file.write(reinterpret_cast<const char*>(verts.data()), verts.size()*sizeof(PLY_Vertex));
        assert(file.good());
    }

    void PPCloud::write_verts(const PPC_Node& node, const std::vector<PLY_Vertex> &verts) {
        assert(node.vert_count == verts.size());
        write_verts(node.vert_offset, verts);
    }

    void PPCloud::read_header() {
        assert(file.good());
        file.seekg(header_section());
        file.read(reinterpret_cast<char*>(&header), sizeof(Header));
        assert(file.good());
    }

    PPC_Node PPCloud::read_node(uint64_t id) {
        file.seekg(nodes_section() + sizeof(PPC_Node)*id);
        PPC_Node result;
        file.read(reinterpret_cast<char*>(&result), sizeof(PPC_Node));
        return result;
    }

    std::vector<PPC_Node> PPCloud::read_nodes(uint64_t base, uint64_t count) {
        std::vector<PPC_Node> result(count);
        file.seekg(nodes_section() + sizeof(PPC_Node) * base);
        file.read(reinterpret_cast<char*>(result.data()), sizeof(PPC_Node) * count);
        return result;
    }

    std::vector<PLY_Vertex> PPCloud::read_verts(uint64_t base, uint64_t count) {
        std::vector<PLY_Vertex> result(count);
        file.seekg(vertices_section() + sizeof(PLY_Vertex)*base);
        file.read(reinterpret_cast<char*>(result.data()), sizeof(PLY_Vertex) * count);
        assert(file.good());
        return result;
    }

    std::vector<PLY_Vertex> PPCloud::read_verts(const PPC_Node& node) {
        return read_verts(node.vert_offset, node.vert_count);
    }

    void PPCloud::flush() {
        assert(!readonly);
        write_header();
        file.flush();
    }
}