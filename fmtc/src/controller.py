#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray

class ControlInput:
    def __init__(self) -> None:
        self.sub = rospy.Subscriber("joy", Joy, self.callback)
        self.pub = rospy.Publisher("control_input", Float32MultiArray, queue_size=10)
        
        self.forward = 1.0
        self.backward = 1.0
        self.angle = 0.0

    def callback(self, data):
        self.forward = data.axes[4]
        self.backward = data.axes[5]
        self.angle = data.axes[0]
    
    def PublishControlInput(self):
        msg = Float32MultiArray()
        msg.data = [self.forward, self.backward, self.angle]
        
        self.pub.publish(msg)

# Intializes everything
def main():
    rospy.init_node('XboxPad', anonymous=True)
    rate = rospy.Rate(100)
    
    control = ControlInput()
    while not rospy.is_shutdown():
        control.PublishControlInput()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
