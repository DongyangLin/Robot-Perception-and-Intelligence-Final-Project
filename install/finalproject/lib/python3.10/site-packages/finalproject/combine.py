from geometry_msgs.msg import PoseStamped
from navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
import rclpy


def initial_pose_setup(navigator,initial_pose):
    ## Set our initial pose
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.27
    initial_pose.pose.position.y = -0.38
    initial_pose.pose.orientation.z = -0.68
    initial_pose.pose.orientation.w = 0.72
    navigator.setInitialPose(initial_pose)


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


def S2A_pose_setup(navigator,goal_pose):
    ## Go to our demos first goal pose (S->A)
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 0.30    #0.32
    goal_pose.pose.position.y = -2.9    #-3.0
    goal_pose.pose.orientation.z = -0.68
    goal_pose.pose.orientation.w = 0.72
    navigator.goToPose(goal_pose)
    check_task_complete(navigator)


def A2B_pose_setup(navigator,goal_pose):
    ## Go to our demos second goal pose (A->B)
    # goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 2.5 #2.6
    goal_pose.pose.position.y = -3.2
    goal_pose.pose.orientation.z = 0.68
    goal_pose.pose.orientation.w = -0.72
    navigator.goToPose(goal_pose)
    check_task_complete(navigator)


def B2S_pose_setup(navigator,goal_pose):
    ## Go to our demos third goal pose (B->S)
    # goal_pose = PoseStamped() 
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 0.27
    goal_pose.pose.position.y = -0.38
    goal_pose.pose.orientation.z = 0.68
    goal_pose.pose.orientation.w = -0.72
    navigator.goToPose(goal_pose)
    check_task_complete(navigator)
    
def main():
    rclpy.init()
    navigator = BasicNavigator()
    initial_pose = PoseStamped()
    goal_pose = PoseStamped()
    ## You may use the navigator to clear or obtain costmaps
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()

    initial_pose_setup(navigator,initial_pose)
    navigator.waitUntilNav2Active()
    navigator.changeMap('/home/tony/map_project/map.yaml')  # map:=$HOME/map_project/map.yaml

    S2A_pose_setup(navigator,goal_pose)
    A2B_pose_setup(navigator,goal_pose)
    
    B2S_pose_setup(navigator,goal_pose)

    exit(0)


if __name__ == '__main__':
    main()