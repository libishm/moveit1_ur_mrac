#!/usr/bin/env python3
from math import pi

import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import Mesh, MeshTriangle, Plane, SolidPrimitive
from std_msgs.msg import Header
from trajectory_tools.trajectory_handler import TrajectoryHandler
from move_group_sequence.move_group_sequence import (Circ, Lin, Ptp, Sequence,
                                                     from_euler)

def robot_program():
    ee_name = "D405"
    ee_name2 = "ur_ee.stl"

    start = (0.0, -pi / 2.0, pi / 2.0, 0.0, pi / 2.0, 0.0)
    pose0 = Pose(
        position=Point(0.8, -0.6, 0.5), orientation=Quaternion(0.0, 1.0, 0.0, 0.0)
    )
    pose1 = Pose(
        position=Point(0.8, -0.6, 0.1), orientation=Quaternion(0.0, 1.0, 0.0, 0.0)
    )
    pose2 = Pose(
        position=Point(0.8, 0.6, 0.5), orientation=Quaternion(0.0, 1.0, 0.0, 0.0)
    )
    pose3 = Pose(
        position=Point(0.8, 0.6, 0.1), orientation=Quaternion(0.0, 1.0, 0.0, 0.0)
    )
    pose4 = Pose(
        position=Point(0.8, 0.6, 0.5), orientation=Quaternion(0.0, 1.0, 0.0, 0.0)
    )

    poses = [pose0, pose1, pose2, pose3, pose4,]

    th = TrajectoryHandler()

    # display pose markers in rviz 
    th.publish_marker_array(poses)

     # Move into position to start reconstruction
    th.sequencer.plan(Ptp(goal=start, vel_scale=0.5, acc_scale=0.7))
    th.sequencer.execute()

    # # attach camera and set new tcp
    # th.attach_camera(ee_name)
    # th.move_group.set_end_effector_link(f"{ee_name}/tcp")
    # rospy.loginfo(
    #     f"{th.name}: end effector link set to {th.move_group.get_end_effector_link()}"
    # )
    th.attach_end_effector(ee_name2)
    th.move_group.set_end_effector_link(f"{ee_name}/tcp")
    rospy.loginfo(
        f"{th.name}: end effector link set to {th.move_group.get_end_effector_link()}"
    )
 

    # set planning pipeline
    # Available planers ompl, chomp, stomp
    th.move_group.set_planning_pipeline_id("stomp")

    # set planner id, see {planner}_planning_pipeline.launch available options
    # th.move_group.set_planner_id('RRTConnect')

    # plan to named position,
    # see srdf and trajectory_handler for available options
   
    # plan to pose

    th.sequencer.plan(Ptp(goal=start, vel_scale=0.5, acc_scale=0.5))
    th.sequencer.execute()

    for pose in poses:
        th.move_group.set_pose_target(pose)
        success, plan = th.move_group.plan()[:2]
        if success:
            th.move_group.execute(plan, wait=True)
        else:
            return rospy.loginfo(f"{th.name}: planning failed, robot program aborted")
    
    th.sequencer.plan(Ptp(goal=start, vel_scale=0.5, acc_scale=0.5))
    th.sequencer.execute()

    return rospy.loginfo(f"{th.name}: robot program completed")


if __name__ == "__main__":

    robot_program()
