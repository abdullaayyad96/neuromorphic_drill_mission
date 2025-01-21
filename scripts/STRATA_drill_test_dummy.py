#!/usr/bin/python
import rospy
import roslib
import numpy as np
import geometry_msgs.msg as geo_msg
import std_msgs.msg as std_msg
import time
from std_msgs.msg import Bool
from Mission_Management.msg import feature_location
from std_srvs.srv import SetBool
from scipy.spatial.transform import Rotation as R
from tf.transformations import quaternion_matrix
from URX.srv import desiredTCP, moveUR, fireDrill
from geometry_msgs.msg import Pose, Twist, PoseStamped, TransformStamped, Transform
import pickle

pose_file = 'pose_data2021-01-19.pickle'


class MissionManagement:
    def __init__(self):
        self.ros_node = rospy.init_node('3DVS_Manager', anonymous=True)

        #define ROS Publisher and Subscriber
        self.adjust_pose_ur_pub = rospy.Publisher('/ur_cmd_adjust_pose', geo_msg.Pose, queue_size=1 )
        self.vel_ur_pub  = rospy.Publisher('/ur_cmd_vel', geo_msg.Twist, queue_size=1 )
        self.detection_mode_pub = rospy.Publisher('/detection_mode', Bool, queue_size=1)
        self.tactile_mode_pub = rospy.Publisher('/tactile_control_mode', Bool, queue_size=1)
        # self.hol_subs = rospy.Subscriber("/hole_pos", feature_location, self.hole_pose_callback) #TODO: load pcd file and read transformation from tf tree
        # self.Plane_Orien_Subscriber = rospy.Subscriber("/icp_transformation", geo_msg.Quaternion, self.plane_orientation_callback) #TODO: read from tf tree


        self.rate = rospy.Rate(100)

        #define ROS service
        self.MM_service_start = rospy.Service('startMM', SetBool, self.start_MM)
        self.startEMVSCall = rospy.ServiceProxy('startEMVS', SetBool)
        self.moveRobotCall = rospy.ServiceProxy('move_ur', moveUR)
        self.setTCPCall = rospy.ServiceProxy('set_TCP', desiredTCP)
        self.fireDrillCall = rospy.ServiceProxy('fire_drill', fireDrill)
        
        self.hole_pose = []
        self.plane_orientation = np.empty((4,))  

        self.tactile_adjusted_pose = geo_msg.Pose()

        self.pose_list = pickle.load(open(pose_file, 'rb'))
        
        rospy.spin()

    def start_MM(self, mess):
        if mess.data == True:
            # print("MissionManagment Started")
            # print(len(self.pose_list['initial_poses']))
            # rospy.sleep(2)
            
            # #Intial Pose
            # hole_pos = self.pose_list['initial_poses'][0]
            # hole_pose = [hole_pos.pose.position.x, hole_pos.pose.position.y, hole_pos.pose.position.z]
            # hole_quat = [hole_pos.pose.orientation.x, hole_pos.pose.orientation.y, hole_pos.pose.orientation.z, hole_pos.pose.orientation.w]

            # hole_rotation = R.from_quat(hole_quat)
            # hole_dcm = hole_rotation.as_dcm()
            # attitude_rot_vec = hole_rotation.as_rotvec()
            # preset_pose = hole_pose + np.matmul(hole_dcm, [0, 0, 0.065])
            # print(preset_pose)
            # print([preset_pose[0], preset_pose[1], preset_pose[2], hole_quat[0], hole_quat[1], hole_quat[2], hole_quat[3]])
            # # raw_input("press Enter to proceed")
            # self.move_robot('pressure_ft', [preset_pose[0], preset_pose[1], preset_pose[2], hole_quat[0], hole_quat[1], hole_quat[2], hole_quat[3]])


            # hole_pos = self.pose_list['initial_poses'][1]
            # hole_pose = [hole_pos.pose.position.x, hole_pos.pose.position.y, hole_pos.pose.position.z]
            # hole_quat = [hole_pos.pose.orientation.x, hole_pos.pose.orientation.y, hole_pos.pose.orientation.z, hole_pos.pose.orientation.w]

            # hole_rotation = R.from_quat(hole_quat)
            # hole_dcm = hole_rotation.as_dcm()
            # attitude_rot_vec = hole_rotation.as_rotvec()
            # preset_pose = hole_pose + np.matmul(hole_dcm, [0, 0, 0.065])
            # print(preset_pose)
            # print([preset_pose[0], preset_pose[1], preset_pose[2], hole_quat[0], hole_quat[1], hole_quat[2], hole_quat[3]])
            # # raw_input("press Enter to proceed")
            # self.move_robot('pressure_ft', [preset_pose[0], preset_pose[1], preset_pose[2], hole_quat[0], hole_quat[1], hole_quat[2], hole_quat[3]])
                        
            # self.performAngleCorrection()
            self.start_hole_tracking()

    def performAngleCorrection(self):

        hole_pos = self.pose_list['angle_correction_poses'][0]
        hole_pose = [hole_pos.pose.position.x, hole_pos.pose.position.y, hole_pos.pose.position.z]
        hole_quat = [hole_pos.pose.orientation.x, hole_pos.pose.orientation.y, hole_pos.pose.orientation.z, hole_pos.pose.orientation.w]

        hole_rotation = R.from_quat(hole_quat)
        hole_dcm = hole_rotation.as_dcm()
        attitude_rot_vec = hole_rotation.as_rotvec()
        preset_pose = hole_pose - np.matmul(hole_dcm, [0, 0, 0.04])
        print(preset_pose)
        print([preset_pose[0], preset_pose[1], preset_pose[2], hole_quat[0], hole_quat[1], hole_quat[2], hole_quat[3]])
        # raw_input("press Enter to proceed")
        self.move_robot('pressure_ft', [preset_pose[0], preset_pose[1], preset_pose[2], hole_quat[0], hole_quat[1], hole_quat[2], hole_quat[3]])

        preset_pose = hole_pose - np.matmul(hole_dcm, [0, 0, 0.0])
        print(preset_pose)
        # raw_input("press Enter to proceed")
        self.move_robot('pressure_ft', [preset_pose[0], preset_pose[1], preset_pose[2], hole_quat[0], hole_quat[1], hole_quat[2], hole_quat[3]])

        hole_pos = self.pose_list['angle_correction_poses'][1]
        hole_pose = [hole_pos.pose.position.x, hole_pos.pose.position.y, hole_pos.pose.position.z]
        hole_quat = [hole_pos.pose.orientation.x, hole_pos.pose.orientation.y, hole_pos.pose.orientation.z, hole_pos.pose.orientation.w]

        hole_rotation = R.from_quat(hole_quat)
        hole_dcm = hole_rotation.as_dcm()
        attitude_rot_vec = hole_rotation.as_rotvec()
        preset_pose = hole_pose - np.matmul(hole_dcm, [0, 0, 0.0])
        print(preset_pose)
        # raw_input("press Enter to proceed")
        self.move_robot('pressure_ft', [preset_pose[0], preset_pose[1], preset_pose[2], hole_quat[0], hole_quat[1], hole_quat[2], hole_quat[3]])

        preset_pose = hole_pose - np.matmul(hole_dcm, [0, 0, 0.04])
        print(preset_pose)
        # raw_input("press Enter to proceed")
        self.move_robot('pressure_ft', [preset_pose[0], preset_pose[1], preset_pose[2], hole_quat[0], hole_quat[1], hole_quat[2], hole_quat[3]])
        
    def start_hole_tracking(self):

        for i in range(11, len(self.pose_list['hole_poses'])):

            hole_pos = self.pose_list['hole_poses'][i]
            hole_pose = [hole_pos.pose.position.x, hole_pos.pose.position.y, hole_pos.pose.position.z]
            hole_quat = [hole_pos.pose.orientation.x, hole_pos.pose.orientation.y, hole_pos.pose.orientation.z, hole_pos.pose.orientation.w]

            hole_rotation = R.from_quat(hole_quat)
            hole_dcm = hole_rotation.as_dcm()
            attitude_rot_vec = hole_rotation.as_rotvec()
            preset_pose = hole_pose - np.matmul(hole_dcm, [0, 0, 0.02])
            print(preset_pose)
            self.move_robot('davis', [preset_pose[0], preset_pose[1], preset_pose[2], hole_quat[0], hole_quat[1], hole_quat[2], hole_quat[3]])
            self.Start_2DVS()

            self.move_robot('pressure_ft', [preset_pose[0], preset_pose[1], preset_pose[2], hole_quat[0], hole_quat[1], hole_quat[2], hole_quat[3]])

            preset_pose = hole_pose - np.matmul(hole_dcm, [0, 0, 0.0])
            print(preset_pose)
            # raw_input("press Enter to proceed")
            self.move_robot('pressure_ft', [preset_pose[0], preset_pose[1], preset_pose[2], hole_quat[0], hole_quat[1], hole_quat[2], hole_quat[3]])
            
            preset_pose = hole_pose - np.matmul(hole_dcm, [0, 0, -0.013])
            print(preset_pose)
            # raw_input("press Enter to proceed")
            self.move_robot('pressure_ft', [preset_pose[0], preset_pose[1], preset_pose[2], hole_quat[0], hole_quat[1], hole_quat[2], hole_quat[3]])

            # raw_input("press Enter to proceed")
            

            rospy.sleep(0.2)
            self.fireDrillCall(True)
            rospy.sleep(0.2)
            
            preset_pose = hole_pose - np.matmul(hole_dcm, [0, 0, 0.02])
            print(preset_pose)
            # raw_input("press Enter to proceed")
            self.move_robot('pressure_ft', [preset_pose[0], preset_pose[1], preset_pose[2], hole_quat[0], hole_quat[1], hole_quat[2], hole_quat[3]])

    def move_robot(self, frame, pose):
            #Intial Pose
            target_pose = geo_msg.Pose()
            target_pose.position.x = pose[0]
            target_pose.position.y = pose[1]
            target_pose.position.z = pose[2]
            target_pose.orientation.x = pose[3]
            target_pose.orientation.y = pose[4]
            target_pose.orientation.z = pose[5]
            target_pose.orientation.w = pose[6]
            
            self.moveRobotCall(frame, target_pose)
            print("robot_moved")
            time.sleep(0.5)
        
            
    def Start_2DVS(self):
        start_VS = Bool()
        start_VS.data = True 
        self.detection_mode_pub.publish(start_VS)
        try:
            Servoing_complete = rospy.wait_for_message('ur_detection_status', Bool, timeout = 10.0)
        except:
            start_VS = Bool()
            start_VS.data = False 
            self.detection_mode_pub.publish(start_VS)
            
        start_VS = Bool()
        start_VS.data = False 
        self.detection_mode_pub.publish(start_VS)
        #print("Servoing Complete: %d", Servoing_complete)


if __name__ == '__main__':
    MissionManagement()
    exit()  


