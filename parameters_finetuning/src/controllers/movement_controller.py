class MovementController:
    def __init__(self, publisher):
        self.publisher = publisher

    def move_in_circle(self, radius, speed):
        move_msg = Twist()
        move_msg.linear.x = speed
        move_msg.angular.z = speed / radius
        self.publisher.publish(move_msg)

    def stop(self):
        move_msg = Twist()
        move_msg.linear.x = 0
        move_msg.angular.z = 0
        self.publisher.publish(move_msg)