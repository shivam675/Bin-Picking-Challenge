#!/usr/bin/env python3
import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
from std_msgs.msg import UInt8, Float32MultiArray
#from PySide import QtCore, QtGui, QtOpenGL
import tf
from tf import TransformListener
from tf.transformations import quaternion_from_euler
import actionlib
import math
from object_msgs.msg import ObjectPose
import geometry_msgs.msg
from gazebo_ros_link_attacher.srv import Attach, AttachRequest
from gazebo_msgs.msg import ModelStates
from scipy.spatial import distance



class Ur5Moveit:

    # Constructor
    def __init__(self):

        self._planning_group = "arm_group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''
        self.OBJECT_IDS = []
        self.OBJECTS = []
        self.OBJECT_INFO = {}
        self.ee_model = 'poke'
        self.ee_link = 'wrist_3_link'
        self.default_can_link = 'link'

        self.object_map = {
            'object_5' : 'coca_cola_clone_0_clone',  
            'object_6' : 'coca_cola_clone_0_clone_2',  
            'object_7' :  'coca_cola_clone_0_clone_4',  
            'object_8' :  'coca_cola',  
            'object_9' : 'coca_cola_clone_0_clone_0',  
            'object_10' : 'coca_cola_clone_0_clone_3' ,  
            'object_11' : 'coca_cola_clone_0_clone_5',  
        }

        self.valid_model_names = [
            'coca_cola', 
            'coca_cola_clone_0_clone',
            'coca_cola_clone_0_clone_0',
            'coca_cola_clone_0_clone_1',
            'coca_cola_clone_0_clone_2',
            'coca_cola_clone_0_clone_3',
            'coca_cola_clone_0_clone_4',
            'coca_cola_clone_0_clone_5',


            ]

        self.model_map = {}

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()



        self.attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        self.attach_srv.wait_for_service()
        rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")



        self.detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
        self.detach_srv.wait_for_service()
        rospy.loginfo("Created ServiceProxy to /link_attacher_node/detach")



        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m') 
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')
    
    def get_detected_object_list(self,):
        msg: Float32MultiArray = rospy.wait_for_message('/objects', Float32MultiArray, timeout=1)

        for idx, val in enumerate(msg.data):
            if idx % 12 == 0:
                self.OBJECT_IDS.append(int(val))

                self.OBJECTS.append('object_' + str(int(val)))

        print(self.OBJECT_IDS)
        print(self.OBJECTS)
    

    def attach(self, model1, link1, model2, link2):
        rospy.loginfo("Detaching")
        req = AttachRequest()
        req.model_name_1 = model1
        req.link_name_1 = link1
        req.model_name_2 = model2
        req.link_name_2 = link2

        self.attach_srv.call(req)

    def detach(self, model1, link1, model2, link2):
        rospy.loginfo("Detaching")
        req = AttachRequest()
        req.model_name_1 = model1
        req.link_name_1 = link1
        req.model_name_2 = model2
        req.link_name_2 = link2
        self.detach_srv.call(req)
    
    def get_closest_model(self,):
        all_models: ModelStates = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=1)
        (trans,rot) = listener.lookupTransform('/world','/tool0', rospy.Time(0))
        trans[2] = trans[2] + 0.05
        least_name = ''
        least_distance = 10
        for idx, name in enumerate(all_models.name):   
            if name not in self.valid_model_names:
                continue
            
            current_object = all_models.pose[idx].position

            _distance = distance.euclidean(trans, [current_object.x, current_object.y, current_object.z])    

            if _distance < least_distance:
                least_distance = _distance
                least_name = name
            print(_distance, name)
            
        print("**************************************************")
        print('Closest Model is {}'.format(least_name))
        print("**************************************************")
        return least_name




    def go_to_pose(self, arg_pose):
        self._planning_group = "arm_group"
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        pose_values = self._group.get_current_pose().pose
        # rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        # rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        list_joint_values = self._group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan
    
    
    def go_to_defined_pose(self, Plan_group, arg_pose_name):
        '''prefined pose combined with plan_group to minimise error '''
        self._planning_group = Plan_group
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._group.set_named_target(arg_pose_name)
        rospy.sleep(1)
        # plan_success, plan, planning_time, error_code = self._group.plan() 
        plan_success, plan, planning_time, error_code = self._group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)
        rospy.sleep(1)
        self._exectute_trajectory_client.wait_for_result()


    def get_object_poses(self, ):
        while self.OBJECTS:
            
            current_object = self.OBJECTS.pop(0)
            object_found = False
            while not object_found and not rospy.is_shutdown():
                try:
                    (trans,rot) = listener.lookupTransform('/world','/' + current_object, rospy.Time(0))
                    self.OBJECT_INFO[current_object] = [trans, rot]
                    object_found = True
                    print(self.OBJECTS)
                except:
                    continue    
        print(self.OBJECT_INFO)


    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')





    def execute(self,):
        for _object in self.OBJECT_INFO:
            # if _object == 'object_5':
            #     continue


            print("*****************************************************")
            print('On Going Object')
            print(_object )
            print("*****************************************************")

            self.go_to_defined_pose("arm_group","detection_pose_simple_world")

            msg = geometry_msgs.msg.Pose()

            msg.position.x = self.OBJECT_INFO[_object][0][0]
            msg.position.y = self.OBJECT_INFO[_object][0][1]
            msg.position.z = self.OBJECT_INFO[_object][0][2] + 0.3
            msg.orientation.x = -0.5069993167594078
            msg.orientation.y = -0.4910390023501438
            msg.orientation.z = 0.4917576173529025
            msg.orientation.w = 0.5099086553019722

            self.go_to_pose(msg)
            rospy.sleep(1)
            
            
            msg.position.z -= 0.1
            self.go_to_pose(msg)
            rospy.sleep(1)


            self.go_to_defined_pose("end_group","grip_2")


            model = self.get_closest_model()
            self.attach(self.ee_model, self.ee_link, model, self.default_can_link)


            msg.position.z += 0.1
            self.go_to_pose(msg)
            rospy.sleep(1)


            self.go_to_defined_pose("arm_group","detection_pose_simple_world")
            self.go_to_defined_pose("arm_group","drop_off_pose")

            self.go_to_defined_pose("end_group","open")
            self.detach(self.ee_model, self.ee_link, model, self.default_can_link)



            
            













def main():
    ur5 = Ur5Moveit()
    
    # Get to detection Pose.
    ur5.go_to_defined_pose("arm_group","detection_pose_simple_world")  # go to Predefined pose # drop object

    # Get object poses via tf

    ur5.get_detected_object_list()
    ur5.get_object_poses()


    ur5.execute()
    

    # best roll pitch yaw values for arm manipulation 
    # roll= -3.12
    # pitch = 0.5 
    # yaw = 1.59
    
    # rospy.sleep(0.5)
    # ur5_pose_1 = geometry_msgs.msg.Pose() 
    # ur5_pose_1.position.x =  0.09
    # ur5_pose_1.position.y =  0.12 # to get better approach in Y  
    # ur5_pose_1.position.z =  1.4 # to get better path planning in Z

    # quaternion = quaternion_from_euler(roll, pitch, yaw)
    # ur5_pose_1.orientation.x = 0.0556
    # ur5_pose_1.orientation.y = 0.9953
    # ur5_pose_1.orientation.z = -0.0043
    # ur5_pose_1.orientation.w = 0.0780
    # rospy.sleep(0.5)
    
    # rospy.loginfo("Attemp in Y")
    # ur5.go_to_pose(ur5_pose_1)    # Attemp to goal in y direction
    
    
    # ur5.go_to_defined_pose("arm_group","detection_pose") # Activating gripper pose
    
    





    del ur5


if __name__ == "__main__":
    rospy.init_node('Main_script', anonymous=True)
    listener = TransformListener()

    main()
