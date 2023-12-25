import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from interbotix_xs_msgs.msg import JointSingleCommand, JointGroupCommand
from sensor_msgs.msg import JointState
import numpy as np
import time
import modern_robotics as mr
from interbotix_xs_modules.xs_robot import mr_descriptions as mrd
from geometry_msgs.msg import PoseArray, Pose, TransformStamped
import math
from pan_tilt_msgs.msg import PanTiltCmdDeg
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry

from rclpy.qos import QoSProfile
from geometry_msgs.msg import Pose, Twist
from rclpy.qos import ReliabilityPolicy
#四元数转欧拉角，只记录航向角
def QtoA(w,x,y,z):
    y = math.atan2(2*(w*z+x*y),1-2*(z*z+y*y))
    angleY = y*180/math.pi 
    return angleY
#计算距离
def distance(x,y):
        return math.sqrt(x*x+y*y)

def create_pose_matrix(theta, translation):
    # 构造绕 z 轴旋转的矩阵
    rotation_matrix_z = np.array([[np.cos(theta), -np.sin(theta), 0],
                                  [np.sin(theta), np.cos(theta), 0],
                                  [0, 0, 1]])

    # 构造平移矩阵
    translation_vector = np.array([[translation[0]],
                                   [translation[1]],
                                   [translation[2]]])

    # 合并旋转矩阵和平移矩阵
    pose_matrix = np.eye(4)
    pose_matrix[:3, :3] = rotation_matrix_z
    pose_matrix[:3, 3:4] = translation_vector

    return pose_matrix


class ArmController(Node):
    def __init__(self):
        super().__init__("ArmController")
        self.cmd_pub = self.create_publisher(JointSingleCommand, "/px100/commands/joint_single", 10)
        self.group_pub = self.create_publisher(JointGroupCommand, "/px100/commands/joint_group", 10)
        self.fb_sub = self.create_subscription(JointState, "/joint_states", self.js_cb, 10)
        self.cam=self.create_subscription(PoseArray,"/aruco_poses",self.cam2arm,10)
        self.pantil_deg_cmd=PanTiltCmdDeg()
        self.pantil_pub=self.create_publisher(PanTiltCmdDeg,"/pan_tilt_cmd_deg",10)
        self.nav_subscriber = self.create_subscription(Int32, 'arm_topic', self.getInstruction, 10)
        self.astar_pub = self.create_publisher(Int32, 'astar_topic', 10)
        self.pub_timer = self.create_timer(0.5, self.control)
        
        self.qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.move_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, self.qos)
        self.move_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.tf_buffer=Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.side_length=0.3
        self.arm_command = JointSingleCommand()
        self.arm_group_command = JointGroupCommand()
        self.instruction=0
        self.num=0
        self.current_step=0
        self.joint_flag=[False,False,False]
        self.cnt = 0
        self.count=0
        self.thred = [0.15,0.15,0.15,0.15]
        self.joint_pos = []
        self.moving_time = 2.0
        self.num_joints = 4
        self.joint_lower_limits = [-1.5, -0.4, -1.1, -1.4]
        self.joint_upper_limits = [1.5, 0.9, 0.8, 1.8]
        self.initial_guesses = [[0.0] * self.num_joints] * 3
        self.initial_guesses[1][0] = np.deg2rad(-30)
        self.initial_guesses[2][0] = np.deg2rad(30)
        self.robot_des: mrd.ModernRoboticsDescription = getattr(mrd, 'px100')
        self.current_pose = None
        self.init_position=None
        self.release_flag=True
        self.up=True
        self.machine_state = "INIT"
        #前进距离
        self.distance_togo = 0
        #旋转角度
        self.rotation_togo = 0
        self.success_catch=False
        self.togo=False
        
        self.rotate_count=0
        
        self.gripper_pressure: float = 0.5
        self.gripper_pressure_lower_limit: int = 0
        self.gripper_pressure_upper_limit: int = 350
        self.gripper_value = self.gripper_pressure_lower_limit + (self.gripper_pressure*(self.gripper_pressure_upper_limit - self.gripper_pressure_lower_limit))
        pass


    def js_cb(self, msg):   # update joint position
        if len(msg.name) == 7:
            self.joint_pos.clear()
            for i in range(7):
                self.joint_pos.append(msg.position[i])


    def cam2arm(self,msg):  # camera return 4 number and 3d coordinate information
        '''
        need to write
        :return:  transform matrix between camera and arm
        '''
        self.cam_x=msg.poses[0].position.x
        self.cam_y=msg.poses[0].position.y
        self.cam_z=msg.poses[0].position.z
        self.cam_rx=msg.poses[0].orientation.x
        self.cam_ry=msg.poses[0].orientation.y
        self.cam_rz=msg.poses[0].orientation.z
        self.cam_rw=msg.poses[0].orientation.w
        transform_stamped_msg = TransformStamped()
        transform_stamped_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
        transform_stamped_msg.header.frame_id = "camera_color_optical_frame"  # 替换为实际的相机坐标系
        transform_stamped_msg.child_frame_id = "object_frame"  # 替换为新的坐标系名
        transform_stamped_msg.transform.translation.x = self.cam_x
        transform_stamped_msg.transform.translation.y = self.cam_y
        transform_stamped_msg.transform.translation.z = self.cam_z
        transform_stamped_msg.transform.rotation.x = self.cam_rx
        transform_stamped_msg.transform.rotation.y = self.cam_ry
        transform_stamped_msg.transform.rotation.z = self.cam_rz
        transform_stamped_msg.transform.rotation.w = self.cam_rw
        self.tf_buffer.set_transform(transform_stamped_msg,self.get_name())


    def getInstruction(self,msg):
        print("Arm successfully get instruction!")
        self.instruction=msg.data

    def set_single_pos(self, name, pos, blocking=True):#return whether it is controlled or not
        '''
        ### @param: name: joint name
        ### @param: pos: radian
        ### @param: blocking - whether the arm need to check current position 
        '''
        self.arm_command.name = name
        self.arm_command.cmd = pos
        self.cmd_pub.publish(self.arm_command)

        thred = self.thred
        if blocking:
            check_pos = None
            cal_name = None
            if len(self.joint_pos) == 7:
                match name:
                    case "waist":
                        check_pos = self.joint_pos[0]+0.07
                        cal_name = 'joint_waist'
                    case "shoulder":
                        check_pos = self.joint_pos[1]
                        cal_name = 'joint_shoulder'
                    case "elbow":
                        check_pos = self.joint_pos[2]
                        cal_name = 'joint_elbow'
                    case "wrist_angle":
                        check_pos = self.joint_pos[3]
                        cal_name = 'joint_wrist_angle'
                    case "gripper":
                        check_pos = self.joint_pos[4]
                        cal_name = 'gripper'
                    case _:
                        print('unvalid name input!')

                match cal_name:
                    case "joint_waist":
                        dis = np.abs(pos-check_pos)
                        if dis < thred[0]:
                            return True
                        else:
                            print('single waist moving...')
                            return False    
                    case "joint_shoulder":
                        dis = np.abs(pos-check_pos)
                        if dis < thred[1]:
                            return True
                        else:
                            print('single shoulder moving...')
                            return False
                    case "joint_elbow":
                        dis = np.abs(pos-check_pos)
                        if dis < thred[2]:
                            return True
                        else:
                            print('single elbow moving...')
                            return False
                    case "joint_wrist_angle":
                        dis = np.abs(pos-check_pos)
                        if dis < thred[3]:
                            return True
                        else:
                            print('single wrist moving...')
                            return False                  
                    case "gripper":
                        return True
        pass


    def set_group_pos(self, pos_list, blocking=True):
        '''
        ### @param: group pos: radian
        ### @param: blocking - whether the arm need to check current position 
        '''
        if len(pos_list) != self.num_joints:
            print('unexpect length of list!')
        else:
            self.arm_group_command.name = "arm"
            self.arm_group_command.cmd = pos_list
            self.group_pub.publish(self.arm_group_command)
            # print(self.arm_group_command)
            thred = self.thred
            if blocking:
                if len(self.joint_pos) == 7:
                    check_pos = self.joint_pos
                    # print('current group pos:', check_pos)
                    if np.abs(pos_list[0] - check_pos[0]) < thred[0] and np.abs(pos_list[1] - check_pos[1]) < thred[1] and np.abs(pos_list[2] - check_pos[2]) < thred[2] and np.abs(pos_list[3] - check_pos[3]) < thred[3]:
                        return True
                    else:
                        if np.abs(pos_list[0] - check_pos[0]) >= thred[0]:
                            print('waist moving...')
                        if np.abs(pos_list[1] - check_pos[1]) >= thred[1]:
                            print('shoulder moving...')
                        if np.abs(pos_list[2] - check_pos[2]) >= thred[2]:
                            print('elbow moving...')
                        if np.abs(pos_list[3] - check_pos[3]) >= thred[3]:
                            print('wrist moving...')
                            return False
            pass

    def joint_to_pose(self, joint_state):
        return mr.FKinSpace(self.robot_des.M, self.robot_des.Slist, joint_state)

    def test(self):
        state=self.set_group_pos([-1.5, 0.0, -1.3, 0.8])
        if state==True:
            print('done!')
            
    def matrix_control(self, T_sd, y,custom_guess: list[float]=None, execute: bool=True):
        if custom_guess is None:
            initial_guesses = self.initial_guesses
        else:
            initial_guesses = [custom_guess]

        for guess in initial_guesses:
            theta_list, success = mr.IKinSpace(
                Slist=self.robot_des.Slist,
                M=self.robot_des.M,
                T=T_sd,
                thetalist0=guess,
                eomg=0.05,
                ev=0.01
            )
            solution_found = True
            print('success',success, solution_found)
            # Check to make sure a solution was found and that no joint limits were violated
            if success:
                # print('success',success)
                theta_list = self._wrap_theta_list(theta_list)
                # solution_found = self._check_joint_limits(theta_list)
                solution_found = True
            else:
                solution_found = False

            if solution_found:
                if execute:
                    joint_list = [theta_list[0]+0.08,theta_list[1]+0.2,theta_list[2], theta_list[3]]
                    print(joint_list)
                    self.T_sb = T_sd
                    return joint_list

        # self.core.get_logger().warn('No valid pose could be found. Will not execute')
        return theta_list, False
    
    def separate_control(self,list,joint_pos):
        if self.num==len(list)+1:
            if self.set_group_pos(joint_pos) ==True:
                self.waist=joint_pos[0]
                print('Done!')
                self.grasp()
                self.num+=1
                time.sleep(1)
                return
        if self.num-1<len(list):
            name=list[self.num-1]
            match name:
                case 'waist':
                    pos=joint_pos[0]
                case 'shoulder':
                    pos=joint_pos[1]
                case 'elbow':
                    pos=joint_pos[2]
                case 'wrist_angle':
                    pos=joint_pos[3]
            if self.set_single_pos (name,pos)==True:
                self.num+=1
    
    def publish_astar(self,x):
        msg=Int32()
        msg.data=x
        self.astar_pub.publish(msg)
        print("Successfully publish astar_instruction!")
    
    
    def init_pos(self,msg):
        self.init_position=msg.pose.pose.position
        self.init_orientation=QtoA(msg.pose.pose.orientation.w,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z)

    def odom_callback(self, msg):
        # 初始化位置和姿态信息 （加入初始信息）
        if  self.init_position is None or self.togo is True:
            self.init_pos(msg)
        # 提取角度和线性位移信息
        self.position = msg.pose.pose.position
        self.orientation = QtoA(msg.pose.pose.orientation.w,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z)
        #计算角度变化量
        self.deltaA=math.fabs(self.init_orientation-self.orientation)
        #计算线性位移变化量
        self.deltax=math.fabs(self.init_position.x-self.position.x)
        self.deltay=math.fabs(self.init_position.y-self.position.y)
        self.deltaz=math.fabs(self.init_position.z-self.position.z)
        #计算线性位移
        self.distance=distance(self.deltax,self.deltay)
        
        
        # 进行两步后停止 
        if self.current_step>1:
            self.rotate(0.0)
            self.move(0.0)
            self.current_step=0
            self.distance_togo=0
            self.rotation_togo=0
        elif self.current_step  == 0:
            # 步骤0：机器人旋转180度
            if self.rotation_togo!=0:
                self.move(0.0)
                if self.deltaA >= self.rotation_togo:
                    self.rotate(0.0)
                    self.current_step += 1
                else:
                    self.rotate(0.1)
        elif self.current_step  == 1:
            if self.distance_togo!=0:
                # 步骤1：机器人向前进1米
                self.rotate(0.0)
                if self.distance >= self.distance_togo:
                    self.move(0.0)
                    self.current_step += 1
                else:
                    self.move(0.01)
            
    def move(self,speed):
        msg=Twist()
        msg.linear.x=speed
        self.move_publisher.publish(msg)
    
    def rotate(self,speed):
        msg=Twist()
        msg.angular.z=speed
        self.move_publisher.publish(msg)
            
        
    def control(self):
        try:
            if self.instruction==1:
                if self.num==0:
                    self.pantil_deg_cmd.pitch=13.0
                    self.pantil_deg_cmd.yaw=0.0
                    self.pantil_deg_cmd.speed=10
                    self.pantil_pub.publish(self.pantil_deg_cmd)
                    self.release()
                    if self.set_group_pos([-1.5, -0.4, 0.5, -0.9]) == True :
                        print('go home pos done!')
                        self.num=1
                        time.sleep(1.0)
                if self.num>=1:
                    list=['waist','waist']
                    if self.num<=len(list)+1:
                        now = rclpy.time.Time()
                        if self.togo==True:
                            if self.rotate_count<5:
                                time.sleep(1)
                                self.rotate(1.2)
                                self.rotate_count+=1
                            elif self.rotate_count>=5 and self.rotate_count<7:
                                time.sleep(1)
                                self.rotate(0.0)
                                self.move(0.1)
                                self.rotate_count+=1
                            elif self.rotate_count>=7 and self.rotate_count<12:
                                time.sleep(1)
                                self.rotate(1.4)
                                self.rotate_count+=1
                            else:
                                self.rotate(0.0)
                                self.move(0.0)
                                time.sleep(3)
                                self.togo=False
                        else:
                            try: 
                                trans = self.tf_buffer.lookup_transform("px100/base_link", "object_frame", now)
                                pos=[]
                                pos.append(trans.transform.translation.x)
                                pos.append(trans.transform.translation.y)
                                pos.append(trans.transform.translation.z)
                                print('x: ', pos[0], 'y: ', pos[1], 'z: ', pos[2])
                                theta=math.atan(pos[1]/pos[0])
                                if pos[0]<0.25:
                                    # 太近  执行后退
                                    self.togo=True
                                    
                                elif pos[0]>0.28 and self.success_catch==False:
                                    #没抓住且距离太远  前进
                                    self.rotate(0.0)
                                    self.move(0.02)
                                else:
                                    #距离合适
                                    self.move(0.0)
                                    if abs(theta)>0.1 and self.success_catch==False:
                                        #角度偏差过大且没抓住  旋转
                                        if theta<0:
                                            self.rotate(-0.01)
                                        elif theta>0:
                                            self.rotate(0.01)
                                        else:
                                            pass
                                    else:
                                        #成功判定可以抓取物体
                                        self.rotate(0.0)
                                        self.move(0.0)
                                        T=create_pose_matrix(theta,pos)
                                        joint_pos=self.matrix_control(T,pos[1])
                                        self.success_catch=True
                                        self.separate_control(list,joint_pos)
                            except:
                                #已经判定可以抓住物体就不在移动
                                if self.success_catch==True:
                                    self.rotate(0.0)
                                    self.move(0.0)
                                #没有看到就旋转
                                else:
                                    self.move(0.0)
                                    self.rotate(0.04)
                
                    else:
                        if self.num==len(list)+2:
                            if self.set_group_pos([-1.5, 0.0, -1.3, 0.8]) ==True:
                                print('back 1 done!')
                                time.sleep(0.2)
                                self.num+=1
                        if self.num==len(list)+3:
                            if self.set_group_pos([-1.5, 0.0, 1.3, -1.0]) ==True:
                                print('The whole process done!')
                                time.sleep(0.2)
                                self.publish_astar(1)
            elif self.instruction==2:
                if self.release_flag==True:
                    self.release()
                    self.release_flag=False
                else:
                    self.publish_astar(2)                          
        except:
            pass
        pass

    def gripper_controller(self, effort, delay: float):
        '''
        effort: release = 1.5
        effort: grasp = -0.6
        '''
        name = 'gripper'
        effort = float(effort)
        if len(self.joint_pos) == 7:
            gripper_state = self.set_single_pos(name, effort)
            time.sleep(delay)
            return gripper_state

    def release(self, delay: float = 1.0) -> None:
        """
        Open the gripper (when in 'pwm' control mode).
        :param delay: (optional) number of seconds to delay before returning control to the user
        """
        state = self.gripper_controller(1.5, delay)
        return state

    def grasp(self, pressure: float = 0.68, delay: float = 1.0) -> None:
        """
        Close the gripper (when in 'pwm' control mode).
        :param delay: (optional) number of seconds to delay before returning control to the user
        """
        state = self.gripper_controller(pressure, delay)
        return state

    def _wrap_theta_list(self, theta_list: list[np.ndarray]) -> list[np.ndarray]:
        REV = 2 * np.pi
        theta_list = (theta_list + np.pi) % REV - np.pi
        for x in range(len(theta_list)):
            if round(theta_list[x], 3) < round(self.joint_lower_limits[x], 3):
                theta_list[x] = self.joint_lower_limits[x]
            elif round(theta_list[x], 3) > round(self.joint_upper_limits[x], 3):
                theta_list[x] = self.joint_upper_limits[x]
        return theta_list


def main():
    rclpy.init(args=None)
    contoller = ArmController()
    rclpy.spin(contoller)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
