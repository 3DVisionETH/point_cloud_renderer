#pragma once

#include <semaphore>
#include <stdexcept>
#include <memory>
#include <thread>

namespace eth_localization {
    template<class T>
    class MPC_Queue {
        struct Node {
            std::binary_semaphore sem_enqueue;
            std::binary_semaphore sem_dequeue;
            T data;

            Node() : sem_enqueue(1), sem_dequeue(0) {}
        };

        std::atomic_flag done = false;
        std::atomic_uint32_t head = 0;
        std::atomic_uint32_t tail = 0;
        std::vector<Node> ring;

    public:
        MPC_Queue(size_t max_chunks_in_ring) : ring(max_chunks_in_ring) {}

        struct Send_Guard {
            Node* node = nullptr;
            operator bool() { return node != nullptr; }
            T* get() { return node ? &node->data : nullptr; }
            Send_Guard(Node* node = nullptr) : node(node) {}
            Send_Guard(Send_Guard&& other) { node = other.node; other.node = nullptr;}
            ~Send_Guard() {
                if(node) {
                    node->sem_dequeue.release();
                }
            }
        };

        struct Recv_Guard {
            Node* node = nullptr;
            operator bool() { return node != nullptr; }
            T* get() { return node ? &node->data : nullptr; }
            Recv_Guard(Node* node = nullptr) : node(node) {}
            Recv_Guard(Recv_Guard&& other) { node = other.node; other.node = nullptr; }
            ~Recv_Guard() {
                if(node) {
                    node->sem_enqueue.release();
                }
            }
        };

        Send_Guard acquire_send() {
            size_t id = (head++)%ring.size();
            ring[id].sem_enqueue.acquire();
            if(done.test()) throw std::invalid_argument("Enqueue on closed queue");
            return {&ring[id]};
        };

        void send(const T& data) {
            Send_Guard guard = acquire_send();
            *guard.get() = data;
        }

        Recv_Guard acquire_block_recv() {
            size_t id = (tail++) % ring.size();
            ring[id].sem_dequeue.acquire();
            if(done.test()) return {};
            return {&ring[id]};
        }

        bool block_recv(T& data) {
            auto guard = acquire_block_recv();
            if(!guard) return false;
            data = std::move(*guard.get());
            return true;
        }

        void signal_done() {
            done.test_and_set();
            for(Node& node : ring) node.sem_dequeue.release(ring.size());
        }

        template<class Fn>
        void for_each(const Fn& fn) {
            while (true) {
                auto recv = this->acquire_block_recv();
                if(!recv) break;
                fn(*recv.get());
            }
        }
    };
}