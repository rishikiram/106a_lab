#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
import sys
import tf.transformations as tr
import trimesh
from scipy.spatial.transform import Rotation

from intera_interface import gripper as robot_gripper

def main():
    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

    #gripper
    # Set up the right gripper
    right_gripper = robot_gripper.Gripper('right_gripper')
    right_gripper.open()

    positions_tuck = [[0.690, 0.160, 0.381]]
    positions = [[0.661, 0.190, -0.130]] # to origin ada
    # positions = [[0.63395, 0.30445, -0.128]] #alice
    # positions = [[0.733, -0.023, -0.129]] #alan origin
    # positions = [[0.7248815, -0.03417877, -0.13834165]]
    quaternion = [0.0, 1.0, 0.0, 0.0]


    #pawn grasps
    # rot_in_obj_frame = np.array([[-7.10196522e-01,  7.04003480e-01, -2.40945606e-08, 2.30034150e-02],
    #                              [ 7.04003480e-01,  7.10196522e-01, -2.43065179e-08, 7.51473078e-03],
    #                              [ 0.00000000e+00, -3.42250590e-08, -1.00000000e+00, 8.55962455e-08],
    #                              [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00, 1.00000000e+00]])
    
    rot_in_obj_frame = np.array([[-0.00336927, -0.91182209,  0.4105717 ,  0.00389547], #gravity grasp 1
                                [-0.99999432,  0.00307219, -0.00138333,  0.00752292],
                                [ 0.        , -0.41057403, -0.91182727,  0.05463253],
                                [ 0.        ,  0.        ,  0.        ,  1.        ]])

    # rot_in_obj_frame = np.array([[-0.70415593,  0.71004412, -0.00133267,  0.00210566], #gravity grasp 2
    #                         [ 0.71004537,  0.70415469, -0.00132161,  0.00178383],
    #                         [ 0.        , -0.00187688, -0.99999824, -0.01708346],
    #                         [ 0.        ,  0.        ,  0.        ,  1.        ]])
    
    rot_in_obj_frame = np.array([[ 0.75357606, -0.65727095, -0.01086376,  0.02163154], #gravity grasp 3
                                [-0.65736073, -0.75347314, -0.01245384, -0.01476865],
                                [ 0.        ,  0.01652633, -0.99986343,  0.01716799],
                                [ 0.        ,  0.        ,  0.        ,  1.        ]])

    rot_in_obj_frame = np.array([[ 5.77808005e-01, -8.02555459e-01, -1.48467656e-01, -5.83426394e-04], #grasp 4
                                [-8.16172720e-01, -5.68167689e-01, -1.05107409e-01, -1.22277125e-02],
                                [ 0.00000000e+00,  1.81907153e-01, -9.83315711e-01, -4.37443452e-02],
                                [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00, 1.00000000e+00]])
    
    rot_in_obj_frame = np.array([[ 0.74555945, -0.6649332 , -0.04477659,  0.01451823], #ferrari 1
                                [-0.66643912, -0.74387475, -0.05009251, -0.00930078],
                                [ 0.        ,  0.06718781, -0.99774035, -0.02108394],
                                [ 0.        ,  0.        ,  0.        ,  1.        ]])
    
    rot_in_obj_frame = np.array([[ 0.75087619, -0.66036331, -0.01025893,  0.02150058], #ferrari 3
                                [-0.66044299, -0.7507856 , -0.01166367, -0.01513573],
                                [ 0.        ,  0.01553341, -0.99987935,  0.01720855],
                                [ 0.        ,  0.        ,  0.        ,  1.        ]])
    
    rot_in_obj_frame = np.array([[-7.03679089e-01,  7.10500662e-01,  4.95468404e-03, 1.78564345e-04], #ferrari 5
                                [ 7.10517938e-01,  7.03661980e-01,  4.90699442e-03, 1.46540765e-02],
                                [ 0.00000000e+00,  6.97334125e-03, -9.99975686e-01, 3.88174224e-02],
                                [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00, 1.00000000e+00]])

    #nozzle grasps
    rot_in_obj_frame = np.array([[-0.70710678,  0.70710678,  0.        , -0.0117176 ], #gravity 1
                                [ 0.70710678,  0.70710678,  0.        ,  0.00193139],
                                [ 0.        ,  0.        , -1.        ,  0.00733273],
                                [ 0.        ,  0.        ,  0.        ,  1.        ]])
    
    rot_in_obj_frame = np.array([[ 7.08323355e-01, -7.05888110e-01, -1.15129406e-06, 3.10943631e-02], #gravity 2
                                [-7.05888110e-01, -7.08323355e-01, -1.15526591e-06, -1.52074116e-03],
                                [ 0.00000000e+00,  1.63098661e-06, -1.00000000e+00, -1.36990676e-02],
                                [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00, 1.00000000e+00]])
    
    rot_in_obj_frame = np.array([[-0.83232528,  0.54824115,  0.08164728, -0.02684983], #gravity 3
                                [ 0.5542875 ,  0.823246  ,  0.1226026 ,  0.01094927],
                                [ 0.        ,  0.14730131, -0.98909167, -0.0178546 ],
                                [ 0.        ,  0.        ,  0.        ,  1.        ]])
    
    rot_in_obj_frame = np.array([[-0.70710678,  0.70710678,  0.        , -0.0117176 ], #ferrari 1
                                [ 0.70710678,  0.70710678,  0.        ,  0.00193139],
                                [ 0.        ,  0.        , -1.        ,  0.00733273],
                                [ 0.        ,  0.        ,  0.        ,  1.        ]])
    
    rot_in_obj_frame = np.array([[-7.07547796e-01,  7.06665492e-01, -4.17580191e-07, 3.11379485e-02], #ferrari 2
                                [ 7.06665492e-01,  7.07547796e-01, -4.18101558e-07, -1.51754568e-03],
                                [ 0.00000000e+00, -5.90916347e-07, -1.00000000e+00, -1.36990406e-02],
                                [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00, 1.00000000e+00]])
    
    rot_in_obj_frame = np.array([[-0.71953395,  0.69439463,  0.00932702, -0.02291174], #ferrari 3
                                [ 0.69445726,  0.71946905,  0.00966381,  0.00795216],
                                [ 0.        ,  0.01343066, -0.9999098 , -0.02156931],
                                [ 0.        ,  0.        ,  0.        ,  1.        ]])
    
    rot_in_obj_frame = np.array([[ 0.92154675,  0.38653008,  0.03668892, -0.01563086], #pawn robust 1
                                [ 0.38826741, -0.91742323, -0.0870806 , -0.02033934],
                                [-0.        ,  0.09449395, -0.99552544,  0.01571587],
                                [ 0.        ,  0.        ,  0.        ,  1.        ]])
    

    rot_in_obj_frame = np.array([[ 0.98334778,  0.14076339, -0.114947  ,  0.02266735], #pawn robust 2
                                [ 0.18173372, -0.76166035,  0.62196975, -0.00761302],
                                [-0.        , -0.63250232, -0.77455847,  0.05461184],
                                [ 0.        ,  0.        ,  0.        ,  1.        ]])
    
    rot_in_obj_frame = np.array([[ 0.94832725, -0.29417668,  0.11889287,  0.03812345], #pawn robust 3
                                [-0.31729392, -0.87923449,  0.3553467 ,  0.00231868],
                                [ 0.        , -0.37470895, -0.92714249,  0.01893623],
                                [ 0.        ,  0.        ,  0.        ,  1.        ]])













    transform_origin_to_gripper = np.array([[-1.0,  0.0,  0.0,   0.685],  #for pawn
                                            [ 0.0,  1.0,  0.0,   0.2],
                                            [ 0.0,  0.0, -1.0,  -0.126],
                                            [ 0.0,  0.0,  0.0,   1.0  ]])
    
    transform_origin_to_gripper = np.array([[-1.0,  0.0,  0.0,   0.656],  #for nozzle
                                            [ 0.0,  1.0,  0.0,   0.184],
                                            [ 0.0,  0.0, -1.0,  -0.153],
                                            [ 0.0,  0.0,  0.0,   1.0  ]])
    
    rot_around_z = np.array([[-1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, -1, 0],
                            [0, 0, 0, 1]])

    pose_gripper = rot_around_z @ transform_origin_to_gripper @ rot_in_obj_frame

    R = pose_gripper[0:3, 0:3]
    print('R', R)
    print('det', np.linalg.det(R))
    R = Rotation.from_matrix(R)
    print('new R', R.as_matrix())
    print('pose gripper', pose_gripper)
    positions = [[transform_origin_to_gripper[0][3], transform_origin_to_gripper[1][3], transform_origin_to_gripper[2][3] + 0]]


    # print('R', R.as_matrix())
    # quaternion = trimesh.transformations.quaternion_from_matrix(R)
    # quaternion = np.array(tr.quaternion_from_matrix(R))
    quaternion = R.as_quat()
    print('positions:', positions)
    print('quaternion0', quaternion)
    print('quat norm0', np.linalg.norm(quaternion))
    # quaternion = quaternion / np.linalg.norm(quaternion)
    # quaternion = quaternion / np.linalg.norm(quaternion)
    index = 0
    while index < len(positions):
        input('Press [ Enter ]: ')
        
        # Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = "right_arm"

        # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
        link = "right_gripper_tip"

        request.ik_request.ik_link_name = link
        # request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"
        
        # Set the desired orientation for the end effector HERE
        request.ik_request.pose_stamped.pose.position.x = positions[index][0]
        request.ik_request.pose_stamped.pose.position.y = positions[index][1]
        request.ik_request.pose_stamped.pose.position.z = positions[index][2]
        request.ik_request.pose_stamped.pose.orientation.x = quaternion[0]
        request.ik_request.pose_stamped.pose.orientation.y = quaternion[1]
        request.ik_request.pose_stamped.pose.orientation.z = quaternion[2]
        request.ik_request.pose_stamped.pose.orientation.w = quaternion[3]
        
        
        try:
            print('Opening...')
            right_gripper.open()
            rospy.sleep(1.0) #flipped z
            # Send the request to the service
            response = compute_ik(request)
            
            # Print the response HERE
            print(response)
            group = MoveGroupCommander("right_arm")

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            # TRY THIS
            # Setting just the position without specifying the orientation
            ###group.set_position_roslaunch intera_examples sawyer_tuck.launchtarget([0.5, 0.5, 0.0])

            # Plan IK
            plan = group.plan()
            user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
            
            # Execute IK if safe
            if user_input == 'y':
                group.execute(plan[1])

            # Close the right gripper
            print('Closing...')
            right_gripper.close()
            rospy.sleep(1.0)

            request.ik_request.pose_stamped.pose.position.x = positions_tuck[0][0]
            request.ik_request.pose_stamped.pose.position.y = positions_tuck[0][1]
            request.ik_request.pose_stamped.pose.position.z = positions_tuck[0][2]
            request.ik_request.pose_stamped.pose.orientation.x = 0.0
            request.ik_request.pose_stamped.pose.orientation.y = 1.0
            request.ik_request.pose_stamped.pose.orientation.z = 0.0
            request.ik_request.pose_stamped.pose.orientation.w = 0.0

            #go up
            response = compute_ik(request)

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            # Print the response HERE
            print(response)
            group = MoveGroupCommander("right_arm")

            # Plan IK
            plan = group.plan()
            user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")

            # Execute IK if safe
            if user_input == 'y':
                group.execute(plan[1])
            
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        if index == 1:
            input("Press <enter> to grab")
            right_gripper.close()
        if index == 3: 
            input("Press <enter> to let go")
            right_gripper.open()
        index += 1
        index = index % 5

# Python's syntax for a main() method
if __name__ == '__main__':
    main()
