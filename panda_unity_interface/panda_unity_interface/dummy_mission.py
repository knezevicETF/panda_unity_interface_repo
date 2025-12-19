#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from panda_unity_interface_msgs.msg import Waypoint, Mission

class DummyMissionPublisher(Node):
    def __init__(self):
        super().__init__('dummy_mission_publisher')
        self.publisher_ = self.create_publisher(Mission, 'mission_command', 10)
        timer_period = 2.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_mission)

        self.mission_sent = False
        self.get_logger().info("Dummy Mission Publisher Initialized")

    def publish_mission(self):
        if self.mission_sent:
            return

        mission = Mission()
        
        # Waypoint 1: MoveJ to a pose
        pose1 = Pose()
        pose1.position.x = 0.4
        pose1.position.y = 0.0
        pose1.position.z = 0.4
        pose1.orientation.w = 0.0  # Identity quaternion
        pose1.orientation.x = 1.0
        waypoint1 = Waypoint(type=0, pose=pose1)

        # Waypoint 2: Open Gripper
        waypoint2 = Waypoint(type=2)

        # Waypoint 3: MoveL to another pose
        pose2 = Pose()
        pose2.position.x = 0.5
        pose2.position.y = -0.2
        pose2.position.z = 0.3
        pose2.orientation.w = 0.0
        pose2.orientation.x = 1.0
        waypoint3 = Waypoint(type=1, pose=pose2)

        # Waypoint 4: Close Gripper
        waypoint4 = Waypoint(type=3)

        mission.waypoints = [waypoint1, waypoint2, waypoint3, waypoint4]

        self.publisher_.publish(mission)
        self.get_logger().info("Published dummy mission with 4 waypoints")
        self.mission_sent = True

def main(args=None):
    rclpy.init(args=args)
    node = DummyMissionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

