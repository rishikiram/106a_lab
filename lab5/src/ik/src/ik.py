#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from intera_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest

def ik_service_client():
    service_name = "ExternalTools/right/PositionKinematicsNode/IKService"
    ik_service_proxy = rospy.ServiceProxy(service_name, SolvePositionIK)
    ik_request = SolvePositionIKRequest()
    header = Header(stamp=rospy.Time.now(), frame_id='base')

    # Create a PoseStamped and specify header (specifying a header is very important!)
    pose_stamped = PoseStamped()
    pose_stamped.header = header

    # Set end effector position: YOUR CODE HERE
    temp_str = input("Please enter (x,y,z): ")
    # 0.5,-0.5,0.1
    #-0.6477526735476687,0.5469809444982225,-2.086964495037678,0.8990112948567274,-0.83259381240628,-1.5741569649523983,-2.789670126919516
    # 0.2,0.3,0.4
    # 1.5330859958508771,-0.06743469194975919,-1.6334076923404885,1.6334807130412583,-1.6379492084449594,-1.6253961655576263,-1.4974895576081113
    # 0.99,0,0
    #-0.5073525106139606,0.13750442348590283,2.7859276258324845,-0.031836719772441076,-2.78301907114717,1.3993082819738156,1.2254600215972704
    # ________________________
#     - Translation: [0.736, -0.061, -0.137]
#     - Rotation: in Quaternion [0.043, 0.997, 0.041, -0.041]
#         in RPY (radian) [3.062, -0.085, 3.058]
#         in RPY (degree) [175.458, -4.881, 175.204]

# - Translation: [0.804, 0.122, 0.368]
# - Rotation: in Quaternion [-0.014, 0.999, -0.027, 0.019]
#             in RPY (radian) [-3.086, 0.037, -3.113]
#             in RPY (degree) [-176.832, 2.109, -178.376]


# - Translation: [0.662, 0.269, -0.135]
# - Rotation: in Quaternion [-0.020, 0.999, 0.022, 0.041]
#             in RPY (radian) [3.099, 0.084, -3.103]
#             in RPY (degree) [177.540, 4.793, -177.811]
    #  ____________________________
    xyz_command = [float(i) for i in temp_str.split(',')]
    pose_stamped.pose.position.x = xyz_command[0]
    pose_stamped.pose.position.y = xyz_command[1]
    pose_stamped.pose.position.z = xyz_command[2]


    # Set end effector quaternion: YOUR CODE HERE
    pose_stamped.pose.orientation.x = 0.0
    pose_stamped.pose.orientation.y = 1.0
    pose_stamped.pose.orientation.z = 0.0
    pose_stamped.pose.orientation.w = 0.0

    # Add desired pose for inverse kinematics
    ik_request.pose_stamp.append(pose_stamped)
    # Request inverse kinematics from base to "right_hand" link
    ik_request.tip_names.append('right_hand')

    rospy.loginfo("Running Simple IK Service Client example.")

    try:
        rospy.wait_for_service(service_name, 5.0)
        response = ik_service_proxy(ik_request)
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logerr("Service call failed: %s" % (e,))
        return

    # Check if result valid, and type of seed ultimately used to get solution
    if (response.result_type[0] > 0):
        rospy.loginfo("SUCCESS!")
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(list(zip(response.joints[0].name, response.joints[0].position)))
        rospy.loginfo("\nIK Joint Solution:\n%s", limb_joints)
        rospy.loginfo("------------------")
        rospy.loginfo("Response Message:\n%s", response)
    else:
        rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
        rospy.logerr("Result Error %d", response.result_type[0])
        return False

    return True


def main():
    rospy.init_node("ik_service_client")

    ik_service_client()

if __name__ == '__main__':
    main()
