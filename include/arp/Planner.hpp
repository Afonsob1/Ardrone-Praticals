#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <Eigen/Core>
#include <queue>

namespace arp {
    /// \brief Planner class
    class Planner {
        
        public:
            Planner(const std::string& filename, int neighbours = 1);
            cv::Mat load_occupancy_map(const std::string& filename);
            std::vector<Eigen::Vector3d> plan_path(Eigen::Vector3d & start, Eigen::Vector3d& goal) const;
            Eigen::Vector3d convertToWorldCoord(const Eigen::Vector3d& pos) const;
            Eigen::Vector3d convertToMapCoord(const Eigen::Vector3d& pos) const;
          
        private:
            /// \brief Open map file.
            std::ifstream _mapFile;

            /// \brief Map size along all the dimensions.
            int _sizes[3];

            /// \brief Map data.
            char* _mapData;
            cv::Mat _wrappedMapData;

            int _neighbours;
    };
}  // namespace arp