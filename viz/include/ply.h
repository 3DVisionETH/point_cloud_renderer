#pragma once

#include <glm/vec3.hpp>
#include "core/math/aabb.h"
#include <mpc_queue.h>

namespace eth_localization {
    struct PLY_Vertex {
        glm::vec3 pos;
        glm::vec3 normal;
        glm::vec<3,char> color;
    };

    struct Convert_Point_Cloud_Options {
        uint64_t read_buffer = 1024*128;
        uint64_t write_buffer = 1024*128;
        uint32_t process_workers = 1;
        uint32_t lods = 4;
        uint32_t verts_per_node = 16*1024;
        uint32_t max_verts_in_memory = 128*1024*1024;
    };

    void partition_into_quadtree(class PPCloud& quadrant, const Convert_Point_Cloud_Options& options);

    void convert_point_cloud(const std::string &dst,
                         const std::string& src,
                         AABB aabb,
                         const Convert_Point_Cloud_Options& options);
}