#!/usr/bin/python
import rospy
import roslib
import numpy as np
import geometry_msgs.msg as geo_msg
import std_msgs.msg as std_msg
import time
from std_msgs.msg import Bool
from Mission_Management.msg import my_msg
from std_srvs.srv import SetBool
from scipy.spatial.transform import Rotation as R
from tf.transformations import quaternion_matrix

initial_pose = []

hole_pose_list = [[0.816925765203, -0.318259696817, 0.240319005167, -0.540986002166, 0.574442299449, -0.408847797923, 0.458468830128],
                 [0.820542970684, -0.2809074544, 0.238890106312, -0.534064819985, 0.545840713783, -0.455327125825, 0.457722505148],
                 [0.819738761297, -0.244739768423, 0.240306478435, -0.540604748391, 0.565669566164, -0.425151097384, 0.454984606692],
                 [0.821547135794, -0.207854622732, 0.240081825185, -0.536196969273, 0.549048542918, -0.452346964118, 0.454335483663],
                 [0.820730158848, -0.170694173258, 0.240696780358, -0.529494458468, 0.532831236323, -0.477029418512, 0.456255877688],
                 [0.821615346891, -0.133635379824, 0.240646390009, -0.542571527185, 0.554302171451, -0.455019929792, 0.437403822692]]
initial_pose = [0.571737126095, -0.358619226217, 0.186093622538, -0.552395999326, 0.575294919232, -0.428328943189, 0.424769034019]

angle_correction_initial_pose = [0.818045003222, -0.44018929975, 0.235850727909, 0.496572214818, -0.465507671654, 0.539384629517, -0.495764928714]
angle_correction_target_pose = [0.82451140027, -0.431216385818, 0.230416951686, 0.551055371928, -0.531376841515, 0.45375368383, -0.456162497123]

class MissionManagement:
    def __init__(self):
        self.ros_node = rospy.init_node('3DVS_Manager', anonymous=True)

        #define ROS Publisher and Subscriber
        self.pose_ur_pub = rospy.Publisher('/ur_cmd_pose', geo_msg.Pose, queue_size=1 )
        self.adjust_pose_ur_pub = rospy.Publisher('/ur_cmd_adjust_pose', geo_msg.Pose, queue_size=1 )
        self.vel_ur_pub  = rospy.Publisher('/ur_cmd_vel', geo_msg.Twist, queue_size=1 )
        # self.Hole_Subscriber = rospy.Subscriber("/hole_pos", my_msg, self.hole_pose_callback)
        # self.Plane_Orien_Subscriber = rospy.Subscriber("/plane_oreintation", geo_msg.Quaternion, self.Plane_Orien_callback)
        self.detection_mode_pub = rospy.Publisher('/detection_mode', Bool, queue_size=1)
        self.tactile_mode_pub = rospy.Publisher('/tactile_control_mode', Bool, queue_size=1)

        self.rate = rospy.Rate(100)

        #define ROS service
        self.MM_service_start = rospy.Service('startMM', SetBool, self.start_MM)
        self.startEMVScall = rospy.ServiceProxy('startEMVS', SetBool)
        
        self.hole_pose = []
        self.plane_orientation = np.empty((4,))  

        self.tactile_adjusted_pose = geo_msg.Pose()
        
        rospy.spin()

    def start_MM(self, mess):
        if mess.data == True:
            print("MissionManagment Started")
                    
            #Intial Pose
            camera_initial_pose = geo_msg.Pose()
            camera_initial_pose.position.x = initial_pose[0]
            camera_initial_pose.position.y = initial_pose[1]
            camera_initial_pose.position.z = initial_pose[2]
            camera_initial_pose.orientation.x = initial_pose[3]
            camera_initial_pose.orientation.y = initial_pose[4]
            camera_initial_pose.orientation.z = initial_pose[5]
            camera_initial_pose.orientation.w = initial_pose[6]
            self.pose_ur_pub.publish(camera_initial_pose)
            time.sleep(5)

            #Velocity Command
            camera_vel_cmd = geo_msg.Twist()
            camera_vel_cmd.linear.x = -0.05
            camera_vel_cmd.linear.y = 0
            camera_vel_cmd.linear.z = 0
            camera_vel_cmd.angular.x = 0
            camera_vel_cmd.angular.y = 0
            camera_vel_cmd.angular.z = 0
            self.vel_ur_pub.publish(camera_vel_cmd)
            time.sleep(3)
            camera_vel_cmd.linear.x = 0
            self.vel_ur_pub.publish(camera_vel_cmd)
            
            self.performAngleCorrection()
            self.start_hole_tracking()

    def performAngleCorrection(self):
        hole_pose = angle_correction_initial_pose[:3]
        hole_quat = angle_correction_initial_pose[3:]

        hole_rotation = R.from_quat(hole_quat)
        hole_dcm = hole_rotation.as_dcm()
        attitude_rot_vec = hole_rotation.as_rotvec()
        preset_pose = hole_pose - np.matmul(hole_dcm, [0, 0, 0.04])
        print(preset_pose)
        raw_input("press Enter to proceed")
        self.move_robot([preset_pose[0], preset_pose[1], preset_pose[2], hole_quat[0], hole_quat[1], hole_quat[2], hole_quat[3]])

        preset_pose = hole_pose - np.matmul(hole_dcm, [0, 0, 0.0])
        print(preset_pose)
        raw_input("press Enter to proceed")
        self.move_robot([preset_pose[0], preset_pose[1], preset_pose[2], hole_quat[0], hole_quat[1], hole_quat[2], hole_quat[3]])
        
        hole_pose = angle_correction_target_pose[:3]
        hole_quat = angle_correction_target_pose[3:]

        hole_rotation = R.from_quat(hole_quat)
        hole_dcm = hole_rotation.as_dcm()
        attitude_rot_vec = hole_rotation.as_rotvec()
        preset_pose = hole_pose - np.matmul(hole_dcm, [0, 0, 0.0])
        print(preset_pose)
        raw_input("press Enter to proceed")
        self.move_robot([preset_pose[0], preset_pose[1], preset_pose[2], hole_quat[0], hole_quat[1], hole_quat[2], hole_quat[3]])

        preset_pose = hole_pose - np.matmul(hole_dcm, [0, 0, 0.04])
        print(preset_pose)
        raw_input("press Enter to proceed")
        self.move_robot([preset_pose[0], preset_pose[1], preset_pose[2], hole_quat[0], hole_quat[1], hole_quat[2], hole_quat[3]])
        
    def start_hole_tracking(self):

        for i in range(len(hole_pose_list)):

            hole_pose = hole_pose_list[i][:3]
            hole_quat = hole_pose_list[i][3:]

            hole_rotation = R.from_quat(hole_quat)
            hole_dcm = hole_rotation.as_dcm()
            attitude_rot_vec = hole_rotation.as_rotvec()
            preset_pose = hole_pose - np.matmul(hole_dcm, [-0.105, -0.085, 0.04])
            print(preset_pose)
            raw_input("press Enter to proceed")
            self.move_robot([preset_pose[0], preset_pose[1], preset_pose[2], hole_quat[0], hole_quat[1], hole_quat[2], hole_quat[3]])
            self.Start_2DVS()

            preset_pose = hole_pose - np.matmul(hole_dcm, [0, 0, 0.04])
            print(preset_pose)
            raw_input("press Enter to proceed")
            self.move_robot([preset_pose[0], preset_pose[1], preset_pose[2], hole_quat[0], hole_quat[1], hole_quat[2], hole_quat[3]])
            preset_pose = hole_pose - np.matmul(hole_dcm, [0, 0, 0.0])
            print(preset_pose)
            raw_input("press Enter to proceed")
            self.move_robot([preset_pose[0], preset_pose[1], preset_pose[2], hole_quat[0], hole_quat[1], hole_quat[2], hole_quat[3]])
            
            preset_pose = hole_pose - np.matmul(hole_dcm, [0, 0, -0.005])
            print(preset_pose)
            raw_input("press Enter to proceed")
            self.move_robot([preset_pose[0], preset_pose[1], preset_pose[2], hole_quat[0], hole_quat[1], hole_quat[2], hole_quat[3]])
            
            preset_pose = hole_pose - np.matmul(hole_dcm, [0, 0, 0.04])
            print(preset_pose)
            raw_input("press Enter to proceed")
            self.move_robot([preset_pose[0], preset_pose[1], preset_pose[2], hole_quat[0], hole_quat[1], hole_quat[2], hole_quat[3]])

    def move_robot(self, pose):
            #Intial Pose
            target_pose = geo_msg.Pose()
            target_pose.position.x = pose[0]
            target_pose.position.y = pose[1]
            target_pose.position.z = pose[2]
            target_pose.orientation.x = pose[3]
            target_pose.orientation.y = pose[4]
            target_pose.orientation.z = pose[5]
            target_pose.orientation.w = pose[6]
            self.pose_ur_pub.publish(target_pose)
            time.sleep(5)
        
            
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


