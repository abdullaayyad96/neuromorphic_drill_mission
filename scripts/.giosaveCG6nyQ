#!/usr/bin/python
from numpy.core.fromnumeric import argsort
from rosgraph.network import use_ipv6
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
from URX.srv import desiredTCP, moveUR, pegHole

import dynamic_reconfigure.client


home_pose = [0.02, -0.72, 0.53, 0, 1/np.sqrt(2), -1/np.sqrt(2), 0] #part on table
# home_pose = [0.2, -0.49, 0.57, 0, 0.7071,  -0.7071, 0] #part with stand
# home_pose = [0.2, -0.49, 0.57, 0.7071, 0, 0,  0.7071] #part with stand - inverted camera
# home_pose = [0.075, -0.44, 0.46, 0.8, 0, 0,  0.7] #STRATA with stand - inverted camera
        
class MissionManagement:
    def __init__(self):
        self.ros_node = rospy.init_node('3DVS_Manager', anonymous=True)

        #define ROS Publisher and Subscriber
        self.adjust_pose_ur_pub = rospy.Publisher('/ur_cmd_adjust_pose', geo_msg.Pose, queue_size=1 )
        self.vel_ur_pub  = rospy.Publisher('/ur_cmd_vel', geo_msg.Twist, queue_size=1 )
        self.detection_mode_pub = rospy.Publisher('/detection_mode', Bool, queue_size=1)
        self.move_pf_to_cam_pub = rospy.Publisher('/move_pressure_to_cam', Bool, queue_size=1)
        self.tactile_mode_pub = rospy.Publisher('/tactile_control_mode', Bool, queue_size=1)
        self.hol_subs = rospy.Subscriber("/hole_pos", feature_location, self.hole_pose_callback) #TODO: load pcd file and read transformation from tf tree
        self.Plane_Orien_Subscriber = rospy.Subscriber("/icp_transformation", geo_msg.Quaternion, self.plane_orientation_callback) #TODO: read from tf tree
        self.stop_scan_subs = rospy.Subscriber("/scan_stop", Bool, self.stop_scan_subs)


        self.MM_service_start = rospy.Service('startMM', SetBool, self.start_MM)
        self.startEMVSCall = rospy.ServiceProxy('startEMVS', SetBool)
        self.startVSCall = rospy.ServiceProxy('startVS', SetBool)
        self.circleExploreCall = rospy.ServiceProxy('explore_srv', SetBool)
        self.circleResetCall  = rospy.ServiceProxy('reset_hough_grid', SetBool)
        self.moveRobotCall = rospy.ServiceProxy('move_ur', moveUR)
        self.setTCPCall = rospy.ServiceProxy('set_TCP', desiredTCP)
        self.insertSplitPinCall = rospy.ServiceProxy('insert_split_pin', pegHole)
        self.visualSplitPinCall = rospy.ServiceProxy('visual_split_pin', pegHole)
        self.retractSplitPinCall = rospy.ServiceProxy('retract_split_pin', pegHole)
        self.fireDrillCall = rospy.ServiceProxy('fire_drill', SetBool)
        self.activateCircleTracker = rospy.ServiceProxy('activate_circle_detector', SetBool)
        
        
        self.rate = rospy.Rate(100)

        #define ROS service
        #Intial Pose
        self.move_pose('davis', home_pose)
        
        self.hole_pose = []
        self.plane_orientation = np.empty((4,))  

        self.tactile_adjusted_pose = geo_msg.Pose()

        self.dvs_client = dynamic_reconfigure.client.Client("davis_ros_driver", timeout=30, config_callback=self.dvs_callback_param)
        
        rospy.spin()
    
    def dvs_callback_param(config, level):
        print("changed dvs params")

    def start_MM(self, mess):
        if mess.data == True:
            print("MissionManagment Started")
                    
            #Intial Pose
            self.move_pose('davis', home_pose)

            #Velocity Command
            self.move_robot('davis', [0.05, 0, 0, 0, 0, 0])
            
            startEMVS = self.startEMVSCall(True)

    
    def stop_scan_subs(self, data):
        #Velocity Command
        self.move_robot('davis', [0, 0, 0, 0, 0, 0])
        print("robot stop")
            

    def hole_pose_callback(self, data): #TODO: make a list of hole poses (now its only of size 3x1   

        for i in range(1, len(data.points)):
            self.hole_pose.append(data.points[i])
        print("length: ", len(self.hole_pose))

    def plane_orientation_callback(self, data):
        self.plane_orientation[0] = data.x
        self.plane_orientation[1] = data.y
        self.plane_orientation[2] = data.z
        self.plane_orientation[3] = data.w

        #only for rotated camera
        r = R.from_quat(self.plane_orientation)
        r2 = R.from_dcm(np.matmul(r.as_dcm(), np.array([[1.,0.,0.], [0.,1.,0.], [0.,0.,1.]])))
        self.plane_orientation = r2.as_quat()

        self.NavigateToFirstHole() 
        # print("subscribed to plane orientation")

    def AdjustDepth(self, plane_Quat, depth_vector_plane):
        Quat = [plane_Quat[0], plane_Quat[1], plane_Quat[2], plane_Quat[3]]
        #Quat = [0.013, 0.70967, -0.401, -0.0200]
        
        PlanetoInertialRotMat = R.from_quat(Quat)
        #PlanetoInertialRotMat = quaternion_matrix([Quat])
        return np.matmul(PlanetoInertialRotMat.as_dcm(), depth_vector_plane)


    def arrange_holes(self):
        ordered_hole_pose = []
        #TODO: remove assumption of two rows
        first_row_poses = []
        second_row_poses = []

        #find mean z of hole poses, this will be used to disreibute the holes on the two poses
        mean_z = 0
        for pose in self.hole_pose:
            mean_z = mean_z + pose.z
        mean_z = mean_z / len(self.hole_pose)

        #distribute according to rows
        for pose in self.hole_pose:
            if pose.z > mean_z:
                first_row_poses.append(pose)
            else:
                second_row_poses.append(pose)

        #distribute among each row
        x_list = []
        for pose in first_row_poses:
            x_list.append(pose.x)
        order_x = np.argsort(x_list)
        print("start ordering")
        print(order_x)
        for i in range(len(order_x)):
            if True:    #i > 0:
                ordered_hole_pose.append(first_row_poses[order_x[i]])

        x_list = []
        for pose in second_row_poses:
            x_list.append(pose.x)
        order_x = np.argsort(x_list)
        for i in range(len(order_x)):
            if True: #i > 6:
                ordered_hole_pose.append(second_row_poses[order_x[i]])      

        self.hole_pose =  ordered_hole_pose      


    def NavigateToFirstHole(self):
        stopEMVS = self.startEMVSCall(False) #TODO: convert this to an action
        
        self.arrange_holes()
        DepthVector = [[0.0], [-0.005], [-0.05]]
        hole_depth_adjust = self.AdjustDepth(self.plane_orientation, DepthVector)
        print(" Adjusted Vector = %d", hole_depth_adjust)
        raw_input("Press Enter to continue...")
        Hole_pose = [0., 0., 0., 0., 0., 0., 0.]
        Hole_pose[0] = self.hole_pose[0].x + hole_depth_adjust[0]
        Hole_pose[1] = self.hole_pose[0].y + hole_depth_adjust[1]
        Hole_pose[2] = self.hole_pose[0].z + hole_depth_adjust[2]
        Hole_pose[3] = self.plane_orientation[0]
        Hole_pose[4] = self.plane_orientation[1]
        Hole_pose[5] = self.plane_orientation[2]
        Hole_pose[6] = self.plane_orientation[3]
        self.move_pose('davis', Hole_pose)

        #self.achieveTactileContact()
    
        #explore_circle = self.circleExploreCall(True)
        self.NavigateToHole() 

    def NavigateToHole(self):
        
        print("Navigate to Hole Started")
        #Hole Pose
        for i in range(len(self.hole_pose)):
            print("hole pose %d is: %d",i, self.hole_pose[i])
            DepthVector = [[0.0], [-0.005], [-0.05]]
            hole_depth_adjust = self.AdjustDepth(self.plane_orientation, DepthVector)
            Hole_pose = [0., 0., 0., 0., 0., 0., 0.]
            Hole_pose[0] = self.hole_pose[i].x + hole_depth_adjust[0]
            Hole_pose[1] = self.hole_pose[i].y + hole_depth_adjust[1]
            Hole_pose[2] = self.hole_pose[i].z + hole_depth_adjust[2]
            Hole_pose[3] = self.plane_orientation[0]
            Hole_pose[4] = self.plane_orientation[1]
            Hole_pose[5] = self.plane_orientation[2]
            Hole_pose[6] = self.plane_orientation[3]
            self.move_pose('davis', Hole_pose)
            self.dvs_client.update_configuration({"pixel_auto_train": True, "exposure": 65000})
            self.circleResetCall(True)
            rospy.sleep(0.5)
            self.activateCircleTracker(True)
            rospy.sleep(0.5)
            DepthVector = [[0.0], [0], [-0.05]]
            hole_depth_adjust = self.AdjustDepth(self.plane_orientation, DepthVector)
            Hole_pose[0] = self.hole_pose[i].x + hole_depth_adjust[0]
            Hole_pose[1] = self.hole_pose[i].y + hole_depth_adjust[1]
            Hole_pose[2] = self.hole_pose[i].z + hole_depth_adjust[2]
            self.move_pose('davis', Hole_pose)
            # explore_circle = self.circleExploreCall(True)

            vs_success = self.startVSCall(True)
            print("VS success: ", bool(vs_success))
            self.activateCircleTracker(False)
            rospy.sleep(0.1)

            movePF = Bool()
            movePF.data = True 

            # usr_input = raw_input("Enter I to split-pin movement, S to Skip")
            if bool(vs_success):#usr_input == 'i':#
                print("i")
                self.visualSplitPinCall(True)

                rospy.sleep(2)
                # if False:#usr_input == 'd':
                drill_success = self.fireDrillCall(True)

                self.retractSplitPinCall(True)

            # user_input = raw_input("Enter to proceed")


            #self.Start_2DVS()

            #self.Adjust_tool_position()

            #self.achieveTactileContact()
        
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


