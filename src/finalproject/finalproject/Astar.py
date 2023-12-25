import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import math
import matplotlib.pyplot as plt
import numpy as np
from std_msgs.msg import Int32

show_animation = True


class AStarPlanner:

    def __init__(self, ox, oy, resolution):
        """Initialize grid map for a star planning
        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]"""
        self.resolution = resolution
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(ox, oy)

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        """
        A star path search

        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        print('minx: ',self.min_x)
        print('miny: ',self.min_y)
        print("start_node:", start_node)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while True:
            if len(open_set) == 0:
                print("Open set is empty..")
                break
        
            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node,open_set[o]))
            print('c_id: ',c_id)
            current = open_set[c_id]

            # show graph
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_grid_position(current.x, self.min_x),
                         self.calc_grid_position(current.y, self.min_y), "xc")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event: [exit(
                                                 0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        """
        calc grid position

        :param index:
        :param min_position:
        :return:
        """
        pos = index + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos))

    def calc_grid_index(self, node):
        return (node.y - self.min_y)* self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        # collision check
        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):
        print("constructing obstacle map")
        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))

        self.x_width = round((self.max_x - self.min_x))
        self.y_width = round((self.max_y - self.min_y) )

        # 使用NumPy创建一个初始化为False的障碍物网格
        self.obstacle_map = np.full((self.x_width, self.y_width), False, dtype=bool)

        # 将障碍物坐标转换为网格索引
        ox_idx = np.round((np.array(ox) - self.min_x)).astype(int)
        oy_idx = np.round((np.array(oy) - self.min_y)).astype(int)
        
        ox_idx = np.clip(ox_idx, 0, self.x_width - 1)
        oy_idx = np.clip(oy_idx, 0, self.y_width - 1)

        # 更新障碍物网格
        self.obstacle_map[ox_idx, oy_idx] = True

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion

class RobotAStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner')
        self.start_pose=None
        self.state=0
        self.origin_x=0
        self.origin_y=0
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.map_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.start_callback, 10)
        self.create_subscription(Int32, 'astar_topic', self.state_callback, 10)
        # self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.path_publisher = self.create_publisher(Path, 'path_topic', 10)
        self.nav_publisher=self.create_publisher(Int32,'nav_topic',10)
        # self.pub_path=self.create_timer(0.5,self.caulculate_path)
    
    def state_callback(self,msg):
        self.state=msg.data
        print("Successfully get astar instruction")
    
    def nav_pub(self,x):
        msg=Int32()
        msg.data=x
        self.nav_publisher.publish(msg)
    
    def map_callback(self, msg):
        self.pixwidth = msg.info.width
        self.pixheight = msg.info.height
        self.resolution = msg.info.resolution #地图分辨率
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.mapMsg = msg
        self.obx=[]
        self.oby=[]
        for i in range(self.pixheight):
            for j in range(self.pixwidth):
                if self.mapMsg.data[i*self.pixwidth+j]>0:
                    self.obx.append(j)
                    self.oby.append(i)

    def worldToMap(self,x,y):
        #将rviz地图坐标转换为栅格坐标
        mx = (int)((x-self.origin_x)/self.resolution)
        my = (int)((y-self.origin_y)/self.resolution)
        return [mx,my]

    
    def create_pose(self,point):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        pose.pose.orientation.z = point[2]
        pose.pose.orientation.w = point[3]
        return pose
    
    def start_callback(self, msg):
        if self.state==0:
        # self.start_pose = self.worldToMap(msg.pose.pose.position.x,msg.pose.pose.position.y)
            self.start_pose_x = msg.pose.pose.position.x
            self.start_pose_y = msg.pose.pose.position.y
            print(self.start_pose_x,self.start_pose_y)
        
    def calculate_path(self,startpoint,goalpoint):
        start=self.create_pose(startpoint)
        self.start_pose = self.worldToMap(start.pose.position.x,start.pose.position.y)
        goal=self.create_pose(goalpoint)
        self.goal_pose = self.worldToMap(goal.pose.position.x,goal.pose.position.y)
        print('start pose: ',self.start_pose)
        print('goal pose: ',self.goal_pose)
        self.Astar()
        
        
    def Astar(self):
        planner=AStarPlanner(self.obx,self.oby,self.resolution)
        rx,ry=planner.planning(self.start_pose[0],self.start_pose[1],self.goal_pose[0],self.goal_pose[1])
        print("rx:",rx)
        print("ry:",ry)
        msg=Path()
        msg.header.frame_id='map'
        for i in range(len(rx)):
            pose=PoseStamped()
            pose.header.frame_id='map'
            pose.header.stamp=self.get_clock().now().to_msg()
            pose.pose.position.x=rx[i]*self.resolution+self.origin_x
            pose.pose.position.y=ry[i]*self.resolution+self.origin_y
            msg.poses.append(pose)
        self.path_publisher.publish(msg)
        print('Complete publishing path')        
        
def main(args=None):
    S=[0.27,-0.38,0.707,-0.707]
    A=[0.20,-3.3,0.707,-0.707]
    A1=[0.20,-3.3,0.707,0.707]
    B=[2.7,-3.2,0.707,0.707]
    rclpy.init(args=args)
    astar_planner = RobotAStarPlanner()
    while astar_planner.origin_x!=-3.8 and astar_planner.origin_y!=-14.5:
        rclpy.spin_once(astar_planner)
    while True:
        rclpy.spin_once(astar_planner)
        if astar_planner.state==0:
            astar_planner.calculate_path(S,A)
            print("S to A")
            break
    while True:
        rclpy.spin_once(astar_planner)
        if astar_planner.state==1:
            astar_planner.calculate_path(A,B)
            astar_planner.nav_pub(1)
            print('A to B')
            break
    while True:
        rclpy.spin_once(astar_planner)
        if astar_planner.state==2:
            astar_planner.calculate_path(B,S)
            astar_planner.nav_pub(2)
            print('B to S')
            break
    astar_planner.destroy_node()
    print("Node has been destroied")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    