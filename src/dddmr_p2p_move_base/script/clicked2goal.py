#! /usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from dddmr_sys_core.action import PToPMoveBase
from geometry_msgs.msg import PoseStamped, PointStamped
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.parameter import Parameter


class Nav2dGoalSub():
    
    def __init__(self, node, m_callback_group):
        self.node_ = node
        self.subscription = self.node_.create_subscription(PoseStamped, 'goal_pose_3d', self.goalCB, 3, callback_group=m_callback_group)
        self.subscription2 = self.node_.create_subscription(PointStamped, 'clicked_point', self.clicked_pointCB, 3, callback_group=m_callback_group)
        self._action_client = ActionClient(self.node_, PToPMoveBase, 'p2p_move_base')
        self.nav2d_goal = []


    def goalCB(self, msg):
        self.node_.get_logger().info("Got goal at: %.2f, %.2f, %.2f, %.2f"  % (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.w))
        self.nav2d_goal = PToPMoveBase.Goal()
        self.nav2d_goal.target_pose = msg

    def clicked_pointCB(self, msg):
        self.node_.get_logger().info("Got clicked point at: %.2f, %.2f, %.2f"  % (msg.point.x, msg.point.y, msg.point.z))
        a_pose = PoseStamped()
        a_pose.pose.position.x = msg.point.x
        a_pose.pose.position.y = msg.point.y
        a_pose.pose.position.z = msg.point.z
        a_pose.pose.orientation.w = 1.0
        a_pose.header.frame_id = "map"
        self.nav2d_goal = PToPMoveBase.Goal()
        self.nav2d_goal.target_pose = a_pose

    def send_goal(self):

        self.node_.get_logger().info("Sending Goal")

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(self.nav2d_goal)
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
        self.node_.get_logger().info("Goal sent. Waiting response")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node_.get_logger().info('Goal rejected :(')
            return

        self.node_.get_logger().info('Goal accepted :)')

def main(args=None):
    rclpy.init(args=args)
    # The relay forwards goal stamps as received and does not read the ROS clock.
    node = rclpy.create_node(
        'clicked2p2p',
        parameter_overrides=[Parameter('use_sim_time', value=False)],
    )
    goal_sub = Nav2dGoalSub(node, MutuallyExclusiveCallbackGroup())
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        while rclpy.ok():
            # Keep the node registered and block until work is ready.
            executor.spin_once()
            if goal_sub.nav2d_goal:
                goal_sub.send_goal()
                goal_sub.nav2d_goal = []
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
