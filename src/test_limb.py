#!/usr/bin/env python

import rospy
from sdl_interface import limb


def main():
    rospy.init_node('test_node')
    rate = rospy.Rate(0.5)
    testLimb = limb.Limb('arm')

    # self._joint_names = {
    #         'arm': ['ur5e_shoulder_pan_joint', 'ur5e_shoulder_lift_joint', 'ur5e_elbow_joint', 'ur5e_wrist_1_joint',
    #                  'ur5e_wrist_2_joint', 'ur5e_wrist_3_joint']
            
    #         }
    positions = {
        'ur5e_shoulder_pan_joint': 1.57,
        'ur5e_shoulder_lift_joint': 0,
        'ur5e_elbow_joint': 0,
        'ur5e_wrist_1_joint': 0,
        'ur5e_wrist_2_joint': 0,
        'ur5e_wrist_3_joint': 0
    }
    testLimb.set_joint_positions(positions)
    while not rospy.is_shutdown():
      print("node is up")
      rate.sleep()

if __name__ == '__main__':
    main()