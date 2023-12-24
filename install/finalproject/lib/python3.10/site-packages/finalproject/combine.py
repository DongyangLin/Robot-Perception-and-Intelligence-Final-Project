from geometry_msgs.msg import PoseStamped
from navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
import rclpy
import time


def create_pose(navigator,point):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = point[0]
    pose.pose.position.y = point[1]
    pose.pose.orientation.z = point[2]
    pose.pose.orientation.w = point[3]
    return pose

def go_to_pose(navigator,point):
    navigator.goToPose(create_pose(navigator,point))
    check_task_complete(navigator)

def initial_pose_setup(navigator,initial_pose):
    navigator.setInitialPose(create_pose(navigator,initial_pose))


def check_task_complete(navigator):
    i=0
    while not navigator.isTaskComplete():
        i=i+1
        feedback = navigator.getFeedback()
        if feedback and i%5==0:
            print(
                'Estimated time of arrival: '+ '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds/ 1e9
                ) + ' seconds.'
            )
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                navigator.cancelTask()
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

def main():
    rclpy.init()
    S=[0.27,-0.38,0.707,-0.707]
    A=[0.27,-3.2,0.707,-0.707]
    A1=[0.27,-3.2,0.0,1.0]
    B=[2.7,-3.2,0.707,0.707]
    navigator = BasicNavigator()
    ## You may use the navigator to clear or obtain costmaps
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()
    initial_pose_setup(navigator,S)
    navigator.publishInfo(0)
    navigator.waitUntilNav2Active()
    navigator.changeMap('/home/tony/map_project/map.yaml')  # map:=$HOME/map_project/map.yaml
    go_to_pose(navigator,A)
    navigator.publishInfo(1)
    time.sleep(1)
    while True:
        rclpy.spin_once(navigator)
        if navigator.instruction==1:
            go_to_pose(navigator,A1)
            go_to_pose(navigator,B)
            navigator.publishInfo(2)
            time.sleep(1)
        elif navigator.instruction==2:
            go_to_pose(navigator,S)
            exit(0)


if __name__ == '__main__':
    main()