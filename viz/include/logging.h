#include <chrono>
#include <string>
#include <iostream>
#include <glm/glm.hpp>

namespace eth_localization {
    using namespace std::chrono;

    class Progess_Log {
        high_resolution_clock::time_point start;
        std::string prefix;
        size_t target = 100;
        size_t last_complete = 0;
        size_t last_perc = 0;
        size_t min_change = 0;
    public:
        Progess_Log() {}
        Progess_Log(const std::string& prefix, size_t target = 100, size_t min_change = 0) : prefix(prefix), target
                (target),min_change(min_change)
        {
            start = high_resolution_clock::now();
        }

        void incr_target(size_t incr) {
            target += incr;
        }

        void update(size_t complete = -1) {
            if(complete == -1) complete = last_complete+1;

            int perc = 100*complete/target;
            if(perc-last_perc > min_change) {
                std::cout
                        << "[" << perc << "] " << prefix << " - Time elapsed "
                        << duration_cast<milliseconds>(high_resolution_clock::now() - start).count() / 1000.0
                        << std::endl;
                last_perc = perc;
            }
            last_complete = complete;
        }
        ~Progess_Log() {
            log(last_complete);
        }
    };

    std::ostream& operator<<(std::ostream& ostream, glm::vec3 vec) {
        ostream << "(" << vec.x << "," << vec.y << "," << vec.z << ")";
        return ostream;
    }
}