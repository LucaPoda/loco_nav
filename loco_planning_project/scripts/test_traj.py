#!/usr/bin/env python3
import rospy
from loco_planning.msg import Reference

def talker():
    # We use a leading slash to ensure it's a GLOBAL topic
    topic_name = '/limo0/ref'
    pub = rospy.Publisher(topic_name, Reference, queue_size=10)
    rospy.init_node('manual_test_node')
    
    # Wait for the publisher to actually connect to a subscriber
    print(f"Waiting for controller to subscribe to {topic_name}...")
    while pub.get_num_connections() == 0 and not rospy.is_shutdown():
        rospy.sleep(0.1)
    
    print("Connected! Sending 2-meter path...")
    rate = rospy.Rate(10) 
    
    for i in range(100): # 10 seconds at 10Hz
        if rospy.is_shutdown(): break
        msg = Reference()
        msg.x_d = 0.02 * i  # Moves 2cm every 0.1s
        msg.y_d = 0.0
        msg.theta_d = 0.0
        msg.v_d = 0.2
        msg.omega_d = 0.0
        msg.plan_finished = False
        pub.publish(msg)
        rate.sleep()

    # Finish signal
    msg.plan_finished = True
    pub.publish(msg)
    print("Done.")

if __name__ == '__main__':
    talker()