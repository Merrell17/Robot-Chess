#!/usr/bin/env python
import sys
import copy
import tf
import tf2_ros
import rospy
import rospkg

from std_msgs.msg import (
    Empty,
    String,
)

from geometry_msgs.msg import (
    Pose,
    Point,
    Quaternion,
)

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)

import baxter_interface
import moveit_commander



class PickAndPlaceMoveIt(object):
    def __init__(self, limb, hover_distance=0.15, verbose=True):
        self._limb_name = limb  # string
        self._hover_distance = hover_distance  # in meters
        self._verbose = verbose  # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")

        self._robot = moveit_commander.RobotCommander()
        self._group = moveit_commander.MoveGroupCommander(limb+"_arm")

    
        # Publisher to let spawn_chessboard know that pnp has taken place and it can spawn the next piece
        self._publish_move = rospy.Publisher("spawn_next", Empty, queue_size=5)
       
        
    

    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to start pose...".format(self._limb_name))

        self.gripper_open()
        self._group.set_pose_target(start_angles)
        plan = self._group.plan()
        self._group.execute(plan)
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)

    def _approach(self, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance

        self._group.set_pose_target(approach)
        plan = self._group.plan()
        self._group.execute(plan)

    def _retract(self):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w

        # servo up from current pose
        self._group.set_pose_target(ik_pose)
        plan = self._group.plan()
        self._group.execute(plan)

    def _servo_to_pose(self, pose):
        # servo down to release
        self._group.set_pose_target(pose)
        plan = self._group.plan()
        self._group.execute(plan)

    def pick(self, pose):
        # open the gripper
        self.gripper_open()
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # close gripper
        self.gripper_close()
        # retract to clear object
        self._retract()

    def place(self, pose):
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract()



# With the correct piece location on the board on and pnp object we handle
# object pick and placement here
def pnp_callback(correct_piece_location, pnp):
    
    # Shift Z coords to convert from rviz to gazebo
    correct_piece_location.position.z = correct_piece_location.position.z - 0.93

    # Match orientation for gripper to be overhead and parallel to chess piece
    overhead_orientation = Quaternion(x=-0.0249590815779, y=0.999649402929, z=0.00737916180073, w=0.00486450832011)
    correct_piece_location.orientation = overhead_orientation
   

    print("Starting Picking")
    pnp.pick(Pose(position=Point(x=0.35, y=0.65, z=0.78-0.93),orientation=overhead_orientation))
    print("Starting Placing")
    pnp.place(correct_piece_location)
    print('Placing Occured')
    
    # Placing has complete, ping spawn_chessboard to start next loop/spawn next piece ready for pnp
    pnp._publish_move.publish(Empty())


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("ik_pick_and_place_moveit")
    rospy.wait_for_message("/robot/sim/started", Empty)

    limb = 'left'
    hover_distance = 0.15  # meters
    
    
    overhead_orientation = Quaternion(x=-0.0249590815779, y=0.999649402929, z=0.00737916180073, w=0.00486450832011)
    
    starting_pose = Pose(
        position=Point(x=0.7, y=0.135, z=0.35),
        orientation=overhead_orientation)
    pnp = PickAndPlaceMoveIt(limb, hover_distance)   

    # Move to desired starting angle
    pnp.move_to_start(starting_pose)
    spawn_chessboard_subscriber = rospy.Subscriber("spawn_chessboard", Pose, pnp_callback, pnp)
    rospy.spin()




if __name__ == '__main__':
    sys.exit(main())
