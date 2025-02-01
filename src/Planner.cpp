#include <arp/Planner.hpp>
#include <ros/ros.h>
#include <Eigen/Core>

#define SCALING_FACTOR 2

namespace arp {
    Planner::Planner(const std::string& filename, int neighbours) : _neighbours(neighbours/2)
    {
        _wrappedMapData = load_occupancy_map(filename);
    }

    cv::Mat Planner::load_occupancy_map(const std::string& filename)
    {

        std::ifstream _mapFile(filename, std::ios::in | std::ios::binary);
        if(!_mapFile.is_open()) {
            ROS_FATAL_STREAM("could not open map file " << filename);
        }

        // first read the map size along all the dimensions:
        if(!_mapFile.read((char*)_sizes, 3*sizeof(int))) {
            ROS_FATAL_STREAM("could not read map file " << filename);
        }

        // now read the map data: don’t forget to delete[] in the end!
        char* _mapData = new char[_sizes[0]*_sizes[1]*_sizes[2]];
        if(!_mapFile.read((char*)_mapData, _sizes[0]*_sizes[1]*_sizes[2])) {
            ROS_FATAL_STREAM("could not read map file " << filename);
        }
        _mapFile.close();

        // now wrap it with a cv::Mat for easier access:
        cv::Mat wrappedMapData(3, _sizes, CV_8SC1, _mapData);

        // reduce map size by merging neighboring cells
        _sizes[0] = _sizes[0] / SCALING_FACTOR;
        _sizes[1] = _sizes[1] / SCALING_FACTOR;
        _sizes[2] = _sizes[2] / SCALING_FACTOR;

        cv::Mat newMap(3, _sizes, CV_8SC1, cv::Scalar(0));

        // compute new occupancy map
        for(int z = 0; z < _sizes[2]; ++z)
        {
            for(int y = 0; y < _sizes[1]; ++y)
            {
                for(int x = 0; x < _sizes[0]; ++x)
                {
                    const int oldX = SCALING_FACTOR * x;
                    const int oldY = SCALING_FACTOR * y;
                    const int oldZ = SCALING_FACTOR * z;

                    int sumVal = 0;
                    for(int dx = 0; dx < SCALING_FACTOR; ++dx)
                    {
                        for(int dy = 0; dy < SCALING_FACTOR; ++dy)
                        {
                            for(int dz = 0; dz < SCALING_FACTOR; ++dz)
                            {
                                sumVal += wrappedMapData.at<char>(
                                    oldX + dx, oldY + dy, oldZ + dz
                                );
                            }
                        }
                    }
                    char avgVal = sumVal / (SCALING_FACTOR * SCALING_FACTOR * SCALING_FACTOR);

                    // store avg value in new map
                    newMap.at<char>(x, y, z) = avgVal;
                }
            }
        }

        // delete the map in the end
        delete[] _mapData;
        return newMap;
    }

    // returns true if there is a collision
    bool Planner::check_collision(const Eigen::Vector3d & pos) const
    {
        int i = std::round(pos.x());
        int j = std::round(pos.y());
        int k = std::round(pos.z());
        
        return _wrappedMapData.at<char>(i, j, k) >= 0;  // occupied if log-odds is 0 or positive.
    }

    // convert a map coordinate (grid index) to world coordinate (in meters)
    Eigen::Vector3d Planner::convertToWorldCoord(const Eigen::Vector3d & pos) const
    {
        double x = (pos[0] - double(_sizes[0] - 1) / 2.0) * (0.1 * SCALING_FACTOR);
        double y = (pos[1] - double(_sizes[1] - 1) / 2.0) * (0.1 * SCALING_FACTOR);
        double z = (pos[2] - double(_sizes[2] - 1) / 2.0) * (0.1 * SCALING_FACTOR);

        return Eigen::Vector3d(x, y, z);
    }

    // convert a world coordinate (in meters) into map (grid) index coordinates
    Eigen::Vector3d Planner::convertToMapCoord(const Eigen::Vector3d & pos) const
    {
        int i = std::round(pos.x() / (0.1 * SCALING_FACTOR) + double(_sizes[0] - 1) / 2.0);
        int j = std::round(pos.y() / (0.1 * SCALING_FACTOR) + double(_sizes[1] - 1) / 2.0);
        int k = std::round(pos.z() / (0.1 * SCALING_FACTOR) + double(_sizes[2] - 1) / 2.0);

        return Eigen::Vector3d(i, j, k);
    }

    bool Planner::check_neighbourhood(Eigen::Vector3d & pos) const
    {
        for (int i = -_neighbours; i <= _neighbours; i++)
            for (int j = -_neighbours; j <= _neighbours; j++)
                for (int k = -_neighbours; k <= _neighbours; k++)
                    if (check_collision(Eigen::Vector3d(pos.x() + i, pos.y() + j, pos.z() + k)))
                        return true;

        return false;
    }

    double distance(const Eigen::Vector3d & pos1, const Eigen::Vector3d & pos2)
    {
        return (pos1 - pos2).norm();
    }

    struct Vector3dHash {
        std::size_t operator()(const Eigen::Vector3d& v) const {
            std::size_t hx = std::hash<double>()(v.x());
            std::size_t hy = std::hash<double>()(v.y());
            std::size_t hz = std::hash<double>()(v.z());
            return hx ^ (hy << 1) ^ (hz << 2);
        }
    };

    std::vector<Eigen::Vector3d> Planner::plan_path(Eigen::Vector3d & start_coord, Eigen::Vector3d & goal_coord) const
    {
        // convert coordinates first
        Eigen::Vector3d start = convertToMapCoord(start_coord);
        Eigen::Vector3d goal = convertToMapCoord(goal_coord);
        
        const int nx = _wrappedMapData.size[0];
        const int ny = _wrappedMapData.size[1];
        const int nz = _wrappedMapData.size[2];
        std::vector<double> dist(nx * ny * nz, std::numeric_limits<double>::infinity());
        std::vector<int> prev_x(nx * ny * nz, -1);
        std::vector<int> prev_y(nx * ny * nz, -1);
        std::vector<int> prev_z(nx * ny * nz, -1);
        
        // helper function to convert 3D coordinates to 1D index
        auto to_index = [&](int x, int y, int z) { return x + nx * (y + ny * z); };
        
        auto cmp = [](const std::pair<double, std::array<int, 3>>& a, 
                    const std::pair<double, std::array<int, 3>>& b) {
            return a.first > b.first;
        };
        
        std::priority_queue<std::pair<double, std::array<int, 3>>,
                            std::vector<std::pair<double, std::array<int, 3>>>,
                            decltype(cmp)> openSet(cmp);
        
        int start_idx = to_index(start.x(), start.y(), start.z());
        dist[start_idx] = 0.0;
        openSet.emplace(distance(start, goal), std::array<int, 3>{(int)start.x(), (int)start.y(), (int)start.z()});
        
        while (!openSet.empty())
        {
            auto curr = openSet.top();
            openSet.pop();

            // check if we reached the goal
            if (
                curr.second[0] == goal.x() &&
                curr.second[1] == goal.y() &&
                curr.second[2] == goal.z()
            ) {
                std::vector<Eigen::Vector3d> path;
                while (curr.second[0] != start.x() || curr.second[1] != start.y() || curr.second[2] != start.z())
                {
                    path.push_back(convertToWorldCoord(Eigen::Vector3d(curr.second[0], curr.second[1], curr.second[2])));
                    int idx = to_index(curr.second[0], curr.second[1], curr.second[2]);
                    curr.second = {prev_x[idx], prev_y[idx], prev_z[idx]};
                }
                std::reverse(path.begin(), path.end());
                return path;
            }

            int curr_idx = to_index(curr.second[0], curr.second[1], curr.second[2]);
            
            // explore neighbors
            for (int i = -_neighbours; i <= _neighbours; i++)
            {
                for (int j = -_neighbours; j <= _neighbours; j++)
                {
                    for (int k = -_neighbours; k <= _neighbours; k++)
                    {
                        int nx_idx = curr.second[0] + i;
                        int ny_idx = curr.second[1] + j;
                        int nz_idx = curr.second[2] + k;
                        
                        if (nx_idx < 0 || nx_idx >= _wrappedMapData.size[0] || 
                            ny_idx < 0 || ny_idx >= _wrappedMapData.size[1] || 
                            nz_idx < 0 || nz_idx >= _wrappedMapData.size[2])
                            continue;
                        
                        Eigen::Vector3d neighbor(nx_idx, ny_idx, nz_idx);
                        if (check_neighbourhood(neighbor))
                            continue;
                        
                        int neighbor_idx = to_index(nx_idx, ny_idx, nz_idx);
                        double alt = dist[curr_idx] + std::sqrt(i*i + j*j + k*k);
                        
                        if (alt < dist[neighbor_idx])
                        {
                            dist[neighbor_idx] = alt;
                            prev_x[neighbor_idx] = curr.second[0];
                            prev_y[neighbor_idx] = curr.second[1];
                            prev_z[neighbor_idx] = curr.second[2];
                            double f = alt + std::sqrt(std::pow(nx_idx - goal.x(), 2) + 
                                                        std::pow(ny_idx - goal.y(), 2) + 
                                                        std::pow(nz_idx - goal.z(), 2));
                            openSet.emplace(f, std::array<int, 3>{nx_idx, ny_idx, nz_idx});
                        }
                    }
                }
            }
        }
        
        // if no path is found, return an empty vector
        return {};
    }
} // namespace arp