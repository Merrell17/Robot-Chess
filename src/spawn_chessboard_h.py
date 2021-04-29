#!/usr/bin/env python
import sys, rospy, tf, rospkg
from gazebo_msgs.srv import *
from geometry_msgs.msg import *
from copy import deepcopy
from std_msgs.msg import (
    Empty,
    String,
)



if __name__ == '__main__':
    rospy.init_node("spawn_chessboard")
    rospy.wait_for_service("gazebo/spawn_sdf_model")

    srv_call = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

    
    

    

    # Table
    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"
    table_xml = ''
    with open(model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xml = table_file.read().replace('\n', '')

    table_pose=Pose(position=Point(x=0.73, y=0.4, z=0.0))
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        spawn_sdf("cafe_table", table_xml, "/", table_pose, "world")
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))
    
    
    # ChessBoard 
    orient = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))
    board_pose = Pose(Point(0.3,0.55,0.78), orient)
    frame_dist = 0.025
    model_path = rospkg.RosPack().get_path('chess_baxter')+"/models/"

    spawned_piece_pose = Pose(position=Point(x=0.35, y=0.65, z=0.78))
    piece_spawned = rospy.Publisher("spawn_chessboard", Pose, queue_size=5)
    
    with open(model_path + "chessboard/model.sdf", "r") as f:
        board_xml = f.read().replace('\n', '')

    # Add chessboard into the simulation
    print srv_call("chessboard", board_xml, "", board_pose, "world")

    # Add chesspieces into the simulation
    origin_piece = 0.03125

    pieces_xml = dict()
    list_pieces = 'rnbqkpRNBQKP'
    for each in list_pieces:
        with open(model_path + each+".sdf", "r") as f:
            pieces_xml[each] = f.read().replace('\n', '')

    # 8 piece boardsetup to be created
    board_setup = ['', '****q***', '**k***p*', '', '*******', '***Q**K*', '', '**R*B*R']

    piece_positionmap = dict()
    piece_names = []
    
    for row, each in enumerate(board_setup):
        # print row
        for col, piece in enumerate(each):
            pose = deepcopy(board_pose)
            pose.position.x = board_pose.position.x + frame_dist + origin_piece + row * (2 * origin_piece)
            pose.position.y = board_pose.position.y - 0.55 + frame_dist + origin_piece + col * (2 * origin_piece)
            pose.position.z += 0.018
            piece_positionmap[str(row)+str(col)] = [pose.position.x, pose.position.y, pose.position.z-0.93]
            if piece in list_pieces:
                
                current_piece = "%s%d" % (piece,col)
                piece_names.append(current_piece)
                
                
                print srv_call(current_piece, pieces_xml[piece], "", spawned_piece_pose, "world")
               
        
                piece_spawned.publish(pose) 
                print(rospy.wait_for_message("spawn_next", Empty)) 
            
            elif not piece=='*' :
                print('error piece in setup: %s' % (piece))
                
    
    rospy.set_param('board_setup', board_setup) # Board setup
    rospy.set_param('list_pieces', list_pieces) # List of unique pieces
    rospy.set_param('piece_target_position_map', piece_positionmap) # 3D positions for each square in the chessboard
    rospy.set_param('piece_names', piece_names) # Pieces that will be part of the game
    rospy.set_param('pieces_xml', pieces_xml) # File paths to Gazebo models, i.e. SDF files
