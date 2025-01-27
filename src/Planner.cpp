#include <arp/Planner.hpp>
#include <ros/ros.h>
#include <unordered_map>
#include <unordered_map>

namespace arp {
    Planner::Planner(int neighbours = 8) : _neighbours(neighbours/2)
    {
        _wrappedMapData = load_occupancy_map();
    }

    cv::Mat Planner::load_occupancy_map() const
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

    bool Planner::check_collision(Eigen::Vector3d & pos) const
    {
        int i = std::round(pos.x/0.1+double(_sizes[0]-1)/2.0)
        int j = std::round(pos.y/0.1+double(_sizes[1]-1)/2.0)
        int k = std::round(pos.z/0.1+double(_sizes[2]-1)/2.0)

        return _wrappedMapData.at<char>(i, j, k) != 0;
    }

    bool Planner::check_neighbourhood(Eigen::Vector3d & pos) const
    {
        int neighbours_limit = std::ceil(_neighbours/2); 
        
        for(int i = -neighbours_limit; i <= neighbours_limit; i++)
            for (int j = -neighbours_limit; j <= neighbours_limit; j++)          
                for (int k = -neighbours_limit; k <= neighbours; k++)
                    if (check_collision(pos.x + i, pos.y + j, pos.z + k))
                        return true;

        return false;
        
    }

    double distance(Eigen::Vector3d & pos1, Eigen::Vector3d & pos2) const
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

    bool Planner::plan_path(Eigen::Vector3d & start, Eigen::Vector3d& goal) const
    {
        int total_size = _wrappedMapData.size[0] * _wrappedMapData.size[1] * _wrappedMapData.size[2];

        std::unordered_map<Eigen::Vector3d, double, Vector3dHash> dist;
        std::unordered_map<Eigen::Vector3d, Eigen::Vector3d, Vector3dHash> prev;
        std::unordered_map<Eigen::Vector3d, Eigen::Vector3d> prev;

        // initialize dist, totDistEst and prev
        for (int i = 0; i < _wrappedMapData.size[0]; i++){
            for (int j = 0; j < _wrappedMapData.size[1]; j++){
                for (int k = 0; k < _wrappedMapData.size[2]; k++){
                    Eigen::Vector3d pos(i, j, k);
                    dist[pos] = std::numeric_limits<double>::infinity();
                    totDistEst[pos] = std::numeric_limits<double>::infinity();
                    prev[pos] = Eigen::Vector3d(-1, -1, -1);
                }
            }
        }

        // initialize start node
        openSet.emplace(0.0, start);
        dist[start] = 0.0;
        totDistEst[start] = distance(start, goal);

        while (!openSet.empty())
        {
            Eigen::Vector3d u = openSet.top().second;
            openSet.pop();

            if (u == goal)
            {
                // Path found, reconstruct path from prev
                return true;
            }
            
            for (int i = -_neighbours; i <= _neighbours; i++)
            {
                for (int j = -_neighbours; j <= _neighbours; j++)
                {
                    for (int k = -_neighbours; k <= _neighbours; k++)
                    {
                        Eigen::Vector3d v(u.x + i, u.y + j, u.z + k);
                        if (v.x < 0 || v.x >= _wrappedMapData.size[0] || v.y < 0 || v.y >= _wrappedMapData.size[1] || v.z < 0 || v.z >= _wrappedMapData.size[2])
                            continue;

                        if (check_neighbourhood(v))
                            continue;

                        double alt = dist[u] + distance(u, v);  // actual distance from start to v

                        if (alt < dist[v])
                        {
                            dist[v] = alt;
                            totDistEst[v] = alt + distance(v, goal);
                            openSet.emplace(totDistEst[v], v);
                            prev[v] = u;
                        }
                    }
                }
            }
        }

        // Path not found
        return false;
    }
}