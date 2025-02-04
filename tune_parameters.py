import rospy
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import PoseStamped
import threading
from rospy.msg import AnyMsg  # Import the generic AnyMsg type


# thread safe
class ThreadSafePose:
    def __init__(self):
        self.pose = None
        self.mutex = threading.Lock()
        
    def updatePose(self, pose):
        self.mutex.acquire()
        self.pose = pose
        self.mutex.release()
        
        
    
    def getPose(self):
        self.mutex.acquire()
        pose = self.pose
        self.mutex.release()
        return pose    
        
pose = ThreadSafePose()

error = 0
last_time = None
start = False  

def gazebo_callback(data):
    global pose, start
    
    for i, name in enumerate(data.name):
        if "sjtu_drone" in name:
            
            pose.updatePose(data.pose[i])
    start = True
    
def rviz_callback(data):
    global pose, error, last_time, start
    if not start:
        return
    if last_time is None:
        last_time = data.header.stamp
        return
    current_time = data.header.stamp
    estimated_pose = data.pose.position
    ground_truth_pose = pose.getPose().position
    error += (estimated_pose.x - ground_truth_pose.x)**2 + (estimated_pose.y - ground_truth_pose.y)**2 + (estimated_pose.z - ground_truth_pose.z)**2
    
    print("Error: ", error/(current_time - last_time).to_sec())


def main():
    rospy.init_node('link_states_listener')
    rospy.Subscriber('/gazebo/link_states', LinkStates, gazebo_callback)
    rospy.Subscriber('/ardrone/vi_ekf_pose', PoseStamped, rviz_callback)
    
    # run the auto pilot in a separate thread
    
    rospy.spin()

if __name__ == "__main__":
    main()