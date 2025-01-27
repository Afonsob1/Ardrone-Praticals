
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>

namespace arp {
    /// \brief Planner class
    class Planner {
        
        public:
            Planner();
            cv::Mat load_occupancy_map() const;
            bool check_collision(double x, double y, double z) const;

        private:
            /// \brief Open map file.
            std::ifstream _mapFile;

            /// \brief Map size along all the dimensions.
            int _sizes[3];

            /// \brief Map data.
            char* _mapData;
            cv::Mat _wrappedMapData;

            int _neighbours;
    }
}  // namespace arp