#!/usr/bin/env python
from __future__ import print_function
import roslib
import rospy
import tf
import tf.msg
import geometry_msgs.msg
import math

class DynamicTransform:

    def __init__(self):
        self.pub_tf = rospy.Publisher("/tf", tf.msg.tfMessage)

        change = 0.0
        while not rospy.is_shutdown():
            rospy.sleep(0.1)

            t = geometry_msgs.msg.TransformStamped()
            t.header.frame_id = "world"
            t.header.stamp = rospy.Time.now()
            t.child_frame_id = "panda_link0"

            t.transform.translation.x = 0.5 * math.cos(change)
            t.transform.translation.y = 0.5 * math.sin(change)
            t.transform.translation.z = 0.

            t.transform.rotation.x = 0.
            t.transform.rotation.y = 0.
            t.transform.rotation.z = 0.
            t.transform.rotation.w = 1.

            tfm = tf.msg.tfMessage([t])
            self.pub_tf.publish(tfm)

            change += 3.14159/50.

if __name__ == "__main__":
    rospy.init_node("to_panda")
    tfb = DynamicTransform()
    rospy.spin()
