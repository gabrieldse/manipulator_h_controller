#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_py.planning import PlanningComponent
from moveit_py.robot_interface import RobotInterface
from moveit_py.trajectory_execution import TrajectoryExecutionManager

class GoalPoseExecutorNode(Node):
    def __init__(self):
        super().__init__('goal_pose_executor_node')

        # Initialize MoveIt 2 components
        self.robot = RobotInterface()
        self.robot.initialize()
        self.arm = PlanningComponent("manipulator_h", self.robot)  # Replace "arm" with your planning group name
        self.executor = TrajectoryExecutionManager(self.robot)

        # Subscribe to the goal pose topic
        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',  # Replace with your goal pose topic name
            self.goal_pose_callback,
            10
        )
        self.get_logger().info("GoalPoseExecutorNode initialized and listening for goal poses.")

    def goal_pose_callback(self, msg: PoseStamped):
        self.get_logger().info(f"Received goal pose: {msg.pose}")

        # Set the pose goal
        self.arm.set_goal(msg.pose)

        # Plan to the goal
        plan = self.arm.plan()
        if plan.success:
            self.get_logger().info("Planning successful. Executing trajectory...")
            self.executor.execute(plan.trajectory)
            self.get_logger().info("Execution complete.")
        else:
            self.get_logger().error("Planning failed. Unable to execute trajectory.")

def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseExecutorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
