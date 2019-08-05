#!/usr/bin/env python
"""
ROS node that publishes the pose (position + quaternion) of the Pozyx

This is an example of how to combine sensor data and positioning into a single
channel output.

Quite overkill using _Pose, as this consists of 7 float64s, while the source
data comes from integers. Suggestions to replace this are quite welcomed.
"""

import pypozyx
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion
import argparse

remote_id = None

# Set serial port or leave it empty to make auto connect
serial_port = ''
# Enable or disable logging
enable_logging = False

def pozyx_pose_pub(port):
    pub = rospy.Publisher('pozyx_pose', Pose, queue_size=40)
    rospy.init_node('pozyx_pose_node')
    try:
        pozyx = pypozyx.PozyxSerial(port)
    except:
        rospy.loginfo("Pozyx not connected")
        return
    while not rospy.is_shutdown():
        coords = pypozyx.Coordinates()
        quat = pypozyx.Quaternion()
        status = pozyx.doPositioning(coords, pypozyx.POZYX_3D, remote_id=remote_id)
        if status == pypozyx.POZYX_SUCCESS:
            pozyx.getQuaternion(quat, remote_id=remote_id)
            if enable_logging:
                rospy.loginfo("POS: %s, QUAT: %s" % (str(coords), str(quat)))
            pub.publish(Point(coords.x, coords.y, coords.z),
                    Quaternion(quat.x, quat.y, quat.z, quat.w))


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--port", type=str,
                        help="sets the uart port of pozyx, use first pozyx device if empty")
    args = parser.parse_args()

    if args.port:
        serial_port = args.port

    # shortcut to not have to find out the port yourself
    if serial_port == '':
        serial_port = pypozyx.get_first_pozyx_serial_port()
        print(serial_port)
        if serial_port is None:
            print("No Pozyx connected. Check your USB cable or your driver!")
            quit()
    
    try:
        pozyx_pose_pub(serial_port)
    except rospy.ROSInterruptException:
        pass
