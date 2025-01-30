#include <arp/Planner.hpp>
#include <ros/ros.h>
#include <Eigen/Core>

namespace arp {
    Planner::Planner(const std::string& filename, int neighbours) : _neighbours(neighbours/2)
    {
        _wrappedMapData = load_occupancy_map(filename);
    }

    cv::Mat Planner::load_occupancy_map(const std::string& filename) const
    {

        std::ifstream _mapFile(filename, std::ios::in | std::ios::binary);
        if(!_mapFile.is_open()) {
          ROS_FATAL_STREAM("could not open map file " << filename);
        }

        // first read the map size along all the dimensions:
        int _sizes[3];
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
        cv::Mat _wrappedMapData(3, _sizes, CV_8SC1, _mapData);

        // delete the map in the end
        delete[] _mapData;
        return _wrappedMapData;
    }

    bool Planner::check_collision(const Eigen::Vector3d & pos) const
    {
        int i = std::round(pos.x()/0.1+double(_sizes[0]-1)/2.0);
        int j = std::round(pos.y()/0.1+double(_sizes[1]-1)/2.0);
        int k = std::round(pos.z()/0.1+double(_sizes[2]-1)/2.0);

        return _wrappedMapData.at<char>(i, j, k) != 0;
    }

    Eigen::Vector3d Planner::convertToWorldCoord(const Eigen::Vector3d & pos) const
    {
        int x = pos[0]*0.1 + double(_sizes[0]-1)/2.0;
        int y = pos[1]*0.1 + double(_sizes[1]-1)/2.0;
        int z = pos[2]*0.1 + double(_sizes[2]-1)/2.0;

        return Eigen::Vector3d(x, y, z);
    }

    Eigen::Vector3d Planner::convertToMapCoord(const Eigen::Vector3d & pos) const
    {
        int i = std::round(pos.x()/0.1+double(_sizes[0]-1)/2.0);
        int j = std::round(pos.y()/0.1+double(_sizes[1]-1)/2.0);
        int k = std::round(pos.z()/0.1+double(_sizes[2]-1)/2.0);

        return Eigen::Vector3d(i, j, k);
    }

    bool Planner::check_neighbourhood(Eigen::Vector3d & pos) const
    {
        for(int i = -_neighbours; i <= _neighbours; i++)
            for (int j = -_neighbours; j <= _neighbours; j++)          
                for (int k = -_neighbours; k <= _neighbours; k++)
                    if (check_collision(Eigen::Vector3d(pos.x() + i, pos.y() + j, pos.z() + k)))
                        return true;

        return false;
        
    }

    double distance(Eigen::Vector3d & pos1, Eigen::Vector3d & pos2)
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

    std::vector<Eigen::Vector3d> Planner::plan_path(Eigen::Vector3d & start_coord, Eigen::Vector3d& goal_coord) const
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
            auto [_, curr] = openSet.top();
            openSet.pop();
            
            // check if we reached the goal
            if (curr[0] == goal.x() && curr[1] == goal.y() && curr[2] == goal.z())
            {
                std::vector<Eigen::Vector3d> path;
                while (curr[0] != start.x() || curr[1] != start.y() || curr[2] != start.z())
                {
                    path.push_back(convertToWorldCoord(Eigen::Vector3d(curr[0], curr[1], curr[2])));
                    int idx = to_index(curr[0], curr[1], curr[2]);
                    curr = {prev_x[idx], prev_y[idx], prev_z[idx]};
                }
                std::reverse(path.begin(), path.end());
                return path;
            }
            
            int curr_idx = to_index(curr[0], curr[1], curr[2]);
            if (dist[curr_idx] < openSet.top().first) continue; // skip if we found a better path
            
            // explore neighbors
            for (int i = -_neighbours; i <= _neighbours; i++)
            {
                for (int j = -_neighbours; j <= _neighbours; j++)
                {
                    for (int k = -_neighbours; k <= _neighbours; k++)
                    {
                        int nx = curr[0] + i;
                        int ny = curr[1] + j;
                        int nz = curr[2] + k;
                        
                        if (nx < 0 || nx >= _wrappedMapData.size[0] || 
                            ny < 0 || ny >= _wrappedMapData.size[1] || 
                            nz < 0 || nz >= _wrappedMapData.size[2])
                            continue;
                        
                        Eigen::Vector3d neighbor(nx, ny, nz);
                        if (check_neighbourhood(neighbor))
                            continue;
                        
                        int neighbor_idx = to_index(nx, ny, nz);
                        double alt = dist[curr_idx] + std::sqrt(i*i + j*j + k*k);
                        
                        if (alt < dist[neighbor_idx])
                        {
                            dist[neighbor_idx] = alt;
                            prev_x[neighbor_idx] = curr[0];
                            prev_y[neighbor_idx] = curr[1];
                            prev_z[neighbor_idx] = curr[2];
                            double f = alt + std::sqrt(std::pow(nx - goal.x(), 2) + 
                                                     std::pow(ny - goal.y(), 2) + 
                                                     std::pow(nz - goal.z(), 2));
                            openSet.emplace(f, std::array<int, 3>{nx, ny, nz});
                        }
                    }
                }
            }
        }
        
        return {};
    }
} // namespace arp