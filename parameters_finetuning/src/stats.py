import rospy
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import PoseStamped, Twist
import threading
from std_msgs.msg import Empty

# Thread-safe pose class
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
class Start:
    def __init__(self):
        self.start = False

pose = ThreadSafePose()
error = 0
last_time = None
start = Start()
process = None

def gazebo_callback(data):
    global pose, start, process
    for i, name in enumerate(data.name):
        if "sjtu_drone" in name:
            pose.updatePose(data.pose[i])
    start.start = True
    
    # start python autopilot.py
    
    import os
    import subprocess
    
    if process is None:
        # Start the autopilot script and print in stdout the output
        process = subprocess.Popen(["python3", "src/autopilot.py"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    
    
    
def rviz_callback(data):
    global pose, error, last_time, start
    if not start.start :
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
    global pubTakeoff_
    rospy.init_node('link_states_listener', anonymous=True)
    rospy.Subscriber('/gazebo/link_states', LinkStates, gazebo_callback)
    rospy.Subscriber('/ardrone/vi_ekf_pose', PoseStamped, rviz_callback)
    
    try:
        rospy.spin()
    
        process.wait()
    except KeyboardInterrupt:
        # Handle the Ctrl+C signal
        process.terminate()
        process.wait()

if __name__ == "__main__":
    main()