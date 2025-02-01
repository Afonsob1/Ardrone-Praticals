#!/usr/bin/env python

import rospy
import numpy as np
import torch
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point

# Load occupancy map from file
def load_occupancy_map(filename):
    try:
        with open(filename, "rb") as file:
            sizes = np.frombuffer(file.read(3 * 4), dtype=np.int32)  # Read dimensions
            total_size = sizes[0] * sizes[1] * sizes[2]
            map_data = np.frombuffer(file.read(total_size), dtype=np.int8)  # Read occupancy data
            occupancy_grid = map_data.reshape(sizes[0], sizes[1], sizes[2])
            return occupancy_grid
    except FileNotFoundError:
        print(f"Error: Could not open map file {filename}")
        return None
    except Exception as e:
        print(f"Error reading map file {filename}: {e}")
        return None

# Convert occupancy map to MarkerArray for RViz
def generate_markers(occupancy_map):
    voxel_size = 0.1
    marker_array = MarkerArray()
    marker_id = 0
    
    # print unique values
    print ("Unique values in occupancy map", np.unique(occupancy_map))
    
    # do upscale in occupancy map merge 2x2x2 to 1 use the max
    upcale = 5
    occupancy_map = torch.tensor(occupancy_map)
    occupancy_map = occupancy_map.unfold(0, upcale, upcale).max(3)[0]
    occupancy_map = occupancy_map.unfold(1, upcale, upcale).max(3)[0]
    occupancy_map = occupancy_map.unfold(2, upcale, upcale).max(3)[0]
    occupancy_map = occupancy_map.numpy()
    
    
    
    
    print ("Occupancy Map ", occupancy_map.shape)
    
    for x, y, z in np.ndindex(occupancy_map.shape):
        

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "occupancy_voxels"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # Set position
        marker.pose.position.x = x * voxel_size*upcale - occupancy_map.shape[0] * voxel_size*upcale / 2
        marker.pose.position.y = y * voxel_size*upcale - occupancy_map.shape[1] * voxel_size*upcale / 2
        marker.pose.position.z = z * voxel_size*upcale - occupancy_map.shape[2] * voxel_size*upcale / 2

        # Set scale (size of each voxel)
        marker.scale.x = voxel_size*upcale
        marker.scale.y = voxel_size*upcale
        marker.scale.z = voxel_size*upcale

        #-5  -4  -3  -2  -1   0   1   2   3 127
        
        if occupancy_map[x, y, z] == 0:
            continue
        elif occupancy_map[x, y, z] > -2  and occupancy_map[x, y, z] < 127:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.5
        elif occupancy_map[x, y, z] == 127:
            continue
        else: 
            continue

        marker_array.markers.append(marker)
        marker_id += 1
    return marker_array

# ROS Node to publish MarkerArray
def occupancy_map_to_rviz(filename):
    rospy.init_node("occupancy_map_publisher", anonymous=True)
    marker_pub = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=10)

    rospy.sleep(1)  # Wait for publisher registration

    occupancy_map = load_occupancy_map(filename)
    if occupancy_map is None:
        return

    print("Map loaded successfully with shape:", occupancy_map.shape)

    marker_array = generate_markers(occupancy_map)
    marker_pub.publish(marker_array)

if __name__ == "__main__":
    try:
        filename = "maps/occupancy-map_skokloster.dat"  # Change to your file path
        occupancy_map_to_rviz(filename)
    except rospy.ROSInterruptException:
        pass
