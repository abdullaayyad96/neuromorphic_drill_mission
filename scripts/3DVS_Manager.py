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
from URX.srv import desiredTCP, moveUR

home_pose = [0.701623982229, -0.258467817455, 0.268140173808, 0.467831979952, -0.508161332871, 0.54726982619, -0.472653187496]

class MissionManagement:
    def __init__(self):
        self.ros_node = rospy.init_node('3DVS_Manager', anonymous=True)

        #define ROS Publisher and Subscriber
        self.adjust_pose_ur_pub = rospy.Publisher('/ur_cmd_adjust_pose', geo_msg.Pose, queue_size=1 )
        self.vel_ur_pub  = rospy.Publisher('/ur_cmd_vel', geo_msg.Twist, queue_size=1 )
        self.detection_mode_pub = rospy.Publisher('/detection_mode', Bool, queue_size=1)
        self.tactile_mode_pub = rospy.Publisher('/tactile_control_mode', Bool, queue_size=1)
        self.hol_subs = rospy.Subscriber("/hole_pos", feature_location, self.hole_pose_callback) #TODO: load pcd file and read transformation from tf tree
        self.Plane_Orien_Subscriber = rospy.Subscriber("/icp_transformation", geo_msg.Quaternion, self.plane_orientation_callback) #TODO: read from tf tree

        self.rate = rospy.Rate(100)

        #define ROS service
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
        self.MM_service_start = rospy.Service('startMM', SetBool, self.start_MM)
        self.startEMVSCall = rospy.ServiceProxy('startEMVS', SetBool)
        self.moveRobotCall = rospy.ServiceProxy('move_ur', moveUR)
        self.setTCPCall = rospy.ServiceProxy('set_TCP', desiredTCP)
        
        self.hole_pose = []
        self.plane_orientation = np.empty((4,))  

        self.tactile_adjusted_pose = geo_msg.Pose()
        
        rospy.spin()

    def start_MM(self, mess):
        if mess.data == True:
            print("MissionManagment Started")
                    
            #Intial Pose
            self.move_pose('davis', home_pose)
            rospy.sleep(10)

            #Velocity Command
            self.move_robot('davis', [-0.05, 0, 0, 0, 0, 0])
            
            startEMVS = self.startEMVSCall(True)
            

    def hole_pose_callback(self, data): #TODO: make a list of hole poses (now its only of size 3x1   
        #Velocity Command
        self.move_robot('davis', [0, 0, 0, 0, 0, 0])

        for i in range(len(data.points)):
            self.hole_pose.append(data.points[i])

    def plane_orientation_callback(self, data):
        self.plane_orientation[0] = data.x
        self.plane_orientation[1] = data.y
        self.plane_orientation[2] = data.z
        self.plane_orientation[3] = data.w

        self.NavigateToFirstHole() 
        # print("subscribed to plane orientation")

    def AdjustDepth(self, plane_Quat, depth_vector_plane):
        Quat = [plane_Quat[0], plane_Quat[1], plane_Quat[2], plane_Quat[3]]
        #Quat = [0.013, 0.70967, -0.401, -0.0200]
        
        PlanetoInertialRotMat = R.from_quat(Quat)
        #PlanetoInertialRotMat = quaternion_matrix([Quat])
        return np.matmul(PlanetoInertialRotMat.as_dcm(), depth_vector_plane)

    def NavigateToFirstHole(self):
        stopEMVS = self.startEMVSCall(False) #TODO: convert this to an action

        DepthVector = [[0], [0], [-0.15]]
        hole_depth_adjust = self.AdjustDepth(self.plane_orientation, DepthVector)
        print(" Adjusted Vector = %d", hole_depth_adjust)
        raw_input("Press Enter to continue...")
        Hole_pose = geo_msg.Pose()
        Hole_pose.position.x = self.hole_pose[0].x + hole_depth_adjust[0]
        Hole_pose.position.y = self.hole_pose[0].y + hole_depth_adjust[1]
        Hole_pose.position.z = self.hole_pose[0].z + hole_depth_adjust[2]
        Hole_pose.orientation.x = self.plane_orientation[0]
        Hole_pose.orientation.y = self.plane_orientation[1]
        Hole_pose.orientation.z = self.plane_orientation[2]
        Hole_pose.orientation.w = self.plane_orientation[3]
        self.pose_ur_pub.publish(Hole_pose)
        time.sleep(5)
        self.achieveTactileContact()

        self.NavigateToHole() 

    def NavigateToHole(self):
        
        print("Navigate to Hole Started")
        #Hole Pose
        for i in range(len(self.hole_pose)):
            print("hole pose %d is: %d",i, self.hole_pose[i])
            DepthVector = [[0], [0], [-0.15]]
            hole_depth_adjust = self.AdjustDepth([self.tactile_adjusted_pose.orientation.x, self.tactile_adjusted_pose.orientation.y, self.tactile_adjusted_pose.orientation.z, self.tactile_adjusted_pose.orientation.w], DepthVector)
            Hole_pose = geo_msg.Pose()
            Hole_pose.position.x = self.hole_pose[i].x + hole_depth_adjust[0]
            Hole_pose.position.y = self.hole_pose[i].y + hole_depth_adjust[1]
            Hole_pose.position.z = self.hole_pose[i].z + hole_depth_adjust[2]
            Hole_pose.orientation.x = self.tactile_adjusted_pose.orientation.x
            Hole_pose.orientation.y = self.tactile_adjusted_pose.orientation.y
            Hole_pose.orientation.z = self.tactile_adjusted_pose.orientation.z
            Hole_pose.orientation.w = self.tactile_adjusted_pose.orientation.w
            time.sleep(5)
            self.pose_ur_pub.publish(Hole_pose)
            time.sleep(5)
            self.Start_2DVS()
            time.sleep(1)
            self.Adjust_tool_position()
            time.sleep(3)
            self.achieveTactileContact()
        
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

    def Adjust_tool_position(self):
        x_correction = -0.003
        y_correction = -0.058
        z_correction = 0 
        Adjust_pose = geo_msg.Pose()
        Adjust_pose.position.x = x_correction
        Adjust_pose.position.y = y_correction
        Adjust_pose.position.z = z_correction
        Adjust_pose.orientation.x = 0
        Adjust_pose.orientation.y = 0
        Adjust_pose.orientation.z = 0
        Adjust_pose.orientation.w = 1
        self.adjust_pose_ur_pub.publish(Adjust_pose)

    def achieveTactileContact(self):
        start_tactile = Bool()
        start_tactile.data = True
        self.tactile_mode_pub.publish(start_tactile)
        
        self.tactile_adjusted_pose = rospy.wait_for_message('tactile_pose', geo_msg.Pose, timeout = 30)

    def move_pose(self, frame="TCP", pose_vector=[]): #TODO: adjust function arguments
        pose_msg = geo_msg.Pose()

        pose_msg.position.x = pose_vector[0]
        pose_msg.position.y = pose_vector[1]
        pose_msg.position.z = pose_vector[2]
        pose_msg.orientation.x = pose_vector[3]
        pose_msg.orientation.y = pose_vector[4]
        pose_msg.orientation.z = pose_vector[5]
        pose_msg.orientation.w = pose_vector[6]
        
        return self.moveRobotCall(frame, pose_msg)

    def move_robot(self, frame="TCP", velocity_vector=[]):
        self.setTCPCall(frame)
        vel_msg = geo_msg.Twist()

        vel_msg.linear.x = velocity_vector[0]
        vel_msg.linear.y = velocity_vector[1]
        vel_msg.linear.z = velocity_vector[2]
        vel_msg.angular.x = velocity_vector[3]
        vel_msg.angular.y = velocity_vector[4]
        vel_msg.angular.z = velocity_vector[5]
        
        self.vel_ur_pub.publish(vel_msg)

    def adjust_pose(self, frame="TCP", pose_vector=[]):
        self.setTCPCall(frame)

        pose_msg = geo_msg.Pose()

        pose_msg.position.x = pose_vector[0]
        pose_msg.position.y = pose_vector[1]
        pose_msg.position.z = pose_vector[2]
        pose_msg.orientation.x = pose_vector[3]
        pose_msg.orientation.y = pose_vector[4]
        pose_msg.orientation.z = pose_vector[5]
        pose_msg.orientation.w = pose_vector[6]
        
        self.adjust_pose_ur_pub.publish(pose_msg)
    


if __name__ == '__main__':
    MissionManagement()
    exit()  


