#include <gtest/gtest.h>
#include "point_cloud.h"
#include <unordered_set>

namespace eth_localization {
    const char *TMP = "tmp/test_file0.pc";

    TEST(PPCloud_Test, alloc_nodes) {
        PPCloud pcloud(TMP, PPCloud::out);
        pcloud.reserve(17, 129);

        std::vector<uint64_t> vertex_per_node(8);
        std::fill(vertex_per_node.begin(), vertex_per_node.end(), 8);

        std::unordered_set<uint32_t> id_set;
        std::unordered_set<uint32_t> vert_set;

        for (int it = 0; it < 2; it++) {
            std::vector<PPC_Node> nodes = pcloud.alloc_nodes_with(vertex_per_node);

            for (uint32_t i = 0; i < vertex_per_node.size(); i++) {
                PPC_Node &node = nodes[i];
                ASSERT_TRUE(id_set.find(node.id) == id_set.end());
                ASSERT_TRUE(node.vert_count == vertex_per_node[i]);
                id_set.insert(node.id);
                for (uint32_t j = 0; j < vertex_per_node[i]; j++) {
                    ASSERT_TRUE(vert_set.find(node.vert_offset + j) == vert_set.end());
                    vert_set.insert(node.vert_offset + j);
                }
            }
        }
    }

    TEST(PPCloud_Test, write_nodes) {
        std::vector<uint64_t> vertex_per_node(8);
        std::fill(vertex_per_node.begin(), vertex_per_node.end(), 8);

        std::vector<PPC_Node> nodes;
        {
            PPCloud pcloud(TMP, PPCloud::out);
            pcloud.reserve(16, 128);

            nodes = pcloud.alloc_nodes_with(vertex_per_node);
            pcloud.write_nodes(nodes);
        }

        {
            uint64_t offset = 2;
            PPCloud pcloud2(TMP, PPCloud::in);
            std::vector<PPC_Node> nodes2 = pcloud2.read_nodes(nodes[offset].id, vertex_per_node.size() - offset);

            ASSERT_EQ(nodes.size(), nodes2.size() + offset);

            for (int i = 0; i < nodes2.size(); i++) {
                ASSERT_EQ(nodes[i + offset].id, nodes2[i].id);
                ASSERT_EQ(nodes[i + offset].vert_offset, nodes2[i].vert_offset);
                ASSERT_EQ(nodes[i + offset].mode, nodes2[i].mode);
                for(int j = 0; j < 4; j++) {
                    ASSERT_EQ(nodes[i + offset].children[j], nodes2[i].children[j]);
                }
            }
        }
    }
}