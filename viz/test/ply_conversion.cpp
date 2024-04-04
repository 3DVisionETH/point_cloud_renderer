\
#include <gtest/gtest.h>
#include "ply.h"
#include "point_cloud.h"

std::ostream& operator<<(std::ostream& o, glm::vec3 v) {
    o << "(" << v.x << "," << v.y << "," << v.z << ")";
    return o;
}

namespace eth_localization {
    static const char* TMP = "tmp/test_file1.pc";

    TEST(PLY_Conversion_Test, quadtree_partition) {
        PPCloud cloud(TMP, PPCloud::Mode(PPCloud::out | PPCloud::in | PPCloud::create));

        int n = 4;
        std::vector<PLY_Vertex> verts(n*n);
        for(uint32_t i = 0; i < n; i++) {
            for(uint32_t j = 0; j < n; j++) {
                PLY_Vertex& v = verts[i*n + j];
                v.pos.x = i + 0.5;
                v.pos.y = j + 0.5;
            }
        }

        cloud.reserve(100, 100);

        PPC_Node root = cloud.alloc_nodes_with({16})[0];
        root.aabb.min = {0,0,0};
        root.aabb.max = {n,n,1};
        cloud.write_nodes(root.id, {root});
        cloud.write_verts(root.vert_offset, verts);
        cloud.set_root(root.id);

        Convert_Point_Cloud_Options options;
        options.verts_per_node = 1;
        options.max_verts_in_memory = 1;
        partition_into_quadtree(cloud, options);

        root = cloud.read_nodes(root.id,1)[0];

        ASSERT_EQ(cloud.node_count(), 21);
        for(int j = 0; j < 4; j++) {
            ASSERT_NE(root.children[j], 0);
            PPC_Node child = cloud.read_nodes(root.children[j], 1)[0];
            for(int k = 0; k < 4; k++) {
                PPC_Node child2 = cloud.read_nodes(child.children[k], 1)[0];
                ASSERT_EQ(child2.vert_count, 1);
                glm::vec3 pos = cloud.read_verts(child2)[0].pos;
                ASSERT_TRUE(child2.aabb.inside(pos));
                std::cout << child2.aabb.min;
                for(int l = 0; l < 4; l++) ASSERT_EQ(child2.children[l], 0);
            }
            ASSERT_EQ(child.vert_count, 4);
        }
    }
}