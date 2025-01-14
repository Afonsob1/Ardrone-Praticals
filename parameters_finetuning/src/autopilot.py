import rospy
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import PoseStamped, Twist
import threading
from std_msgs.msg import Empty


def main():
    rospy.init_node('autopiloto_listener', anonymous=True)
    
    pubMove_ = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    pubTakeoff_ = rospy.Publisher('/drone/takeoff', Empty, queue_size=10)    
        
    pubTakeoff_.publish(Empty())
    print("Takeoff")
    
    rospy.spin()
    
    moveMsg = Twist()
    count = 10
    # every 0.1 seconds send a command to move the drone in a circle
    for _ in range(count):
        moveMsg.linear.x = 1
        moveMsg.angular.z = 1
        pubMove_.publish(moveMsg)
        print("Moving the drone")
        rospy.sleep(0.1)
        
        print("Moving the drone")
    
    # Stop the autopilot
    moveMsg.linear.x = 0
    moveMsg.angular.z = 0
    pubMove_.publish(moveMsg)

if __name__ == "__main__":
    main()