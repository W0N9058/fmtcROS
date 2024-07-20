#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray, Bool

class ControlInput:
    def __init__(self) -> None:
        self.sub = rospy.Subscriber("joy", Joy, self.callback)
        self.pub = rospy.Publisher("control_input", Float32MultiArray, queue_size=10)
        self.save_pub = rospy.Publisher("save_command", Bool, queue_size=10)
        
        self.forward = 1.0
        self.backward = 1.0
        self.angle = 0.0
        self.save = False
        self.last_save_state = False

    def callback(self, data):
        self.forward = data.axes[4]
        self.backward = data.axes[5]
        self.angle = data.axes[0]
        self.save = data.buttons[0]  # Assuming button 0 is the save button
    
    def PublishControlInput(self):
        msg = Float32MultiArray()
        msg.data = [self.forward, self.backward, self.angle]
        self.pub.publish(msg)

    def PublishSaveCommand(self):
        if self.save != self.last_save_state:
            save_msg = Bool()
            save_msg.data = self.save
            self.save_pub.publish(save_msg)
            self.last_save_state = self.save

# Intializes everything
def main():
    rospy.init_node('XboxPad', anonymous=True)
    rate = rospy.Rate(100)
    
    control = ControlInput()
    while not rospy.is_shutdown():
        control.PublishControlInput()
        control.PublishSaveCommand()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

