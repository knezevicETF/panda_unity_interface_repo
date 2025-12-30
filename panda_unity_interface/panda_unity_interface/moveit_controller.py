#!/usr/bin/env python3

from threading import Thread
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from panda_unity_interface_msgs.msg import Waypoint, Mission
from pymoveit2 import MoveIt2, GripperInterface, MoveIt2Servo
from pymoveit2.robots import panda as robot


class FrankaMoveitController(Node):
    """
    ROS 2 node to control Franka FR3 robot using MoveIt2 via pymoveit2.
    
    Action types:
    0 - MoveJ (PTP - Point-to-Point joint space motion)
    1 - MoveL (Linear Cartesian motion)
    2 - Open Gripper
    3 - Close Gripper
    """
    
    def __init__(self):
        super().__init__('franka_moveit_controller')
        
        # Parameters
        self.declare_parameter('joint_names', robot.joint_names())
        self.declare_parameter('base_link_name', robot.base_link_name())
        self.declare_parameter('end_effector_name', robot.end_effector_name())
        self.declare_parameter('group_name', robot.MOVE_GROUP_ARM)
        
        # Get parameters
        joint_names = self.get_parameter('joint_names').value
        base_link_name = self.get_parameter('base_link_name').value
        end_effector_name = self.get_parameter('end_effector_name').value
        group_name = self.get_parameter('group_name').value
        
        # Create callback group that allows execution of callbacks in parallel
        callback_group = ReentrantCallbackGroup()
        
        # Initialize MoveIt2 interface
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=joint_names,
            base_link_name=base_link_name,
            end_effector_name=end_effector_name,
            group_name=group_name,
            callback_group=callback_group
        )
        
                # Initialize Gripper interface
        self.gripper = GripperInterface(
            node=self,
            gripper_joint_names=robot.gripper_joint_names(),
            open_gripper_joint_positions=robot.OPEN_GRIPPER_JOINT_POSITIONS,
            closed_gripper_joint_positions=robot.CLOSED_GRIPPER_JOINT_POSITIONS,
            gripper_group_name=robot.MOVE_GROUP_GRIPPER,
            callback_group=callback_group,
            gripper_command_action_name="gripper_action_controller/gripper_cmd",
        )

        # DragToTeach State
        self.is_teaching = False
        
        # Initialize MoveIt2Servo
        self.servo = MoveIt2Servo(
            node=self,
            frame_id=base_link_name,
            callback_group=callback_group,
        )

        # Subscriber for VR Controller Delta (Twist)
        self.delta_subscription = self.create_subscription(
            Twist,
            'vr_controller_delta',
            self.delta_callback,
            10,
            callback_group=callback_group
        )

        # Servo Service Clients
        self.servo_start_client = self.create_client(Trigger, '/servo_node/start_servo', callback_group=callback_group)
        self.servo_stop_client = self.create_client(Trigger, '/servo_node/stop_servo', callback_group=callback_group)
        
        # Subscriber for Teach Mode (Bool)
        self.teach_mode_subscription = self.create_subscription(
            Bool,
            'teach_mode',
            self.teach_mode_callback,
            10,
            callback_group=callback_group
        )
        
        # Set velocity and acceleration scaling
        self.moveit2.max_velocity = 0.5
        self.moveit2.max_acceleration = 0.5
        
        # Subscriber for waypoint commands
        self.subscription = self.create_subscription(
            Mission,
            'mission_command',
            self.command_callback,
            10,
            callback_group=callback_group
        )
        
        self.get_logger().info('==============================================')
        self.get_logger().info('Franka MoveIt2 Controller Initialized')
        self.get_logger().info('==============================================')
        self.get_logger().info(f'Group name: {group_name}')
        self.get_logger().info(f'Base link: {base_link_name}')
        self.get_logger().info(f'End effector: {end_effector_name}')
        self.get_logger().info(f'Joint names: {joint_names}')
        self.get_logger().info('==============================================')
        self.get_logger().info('Action types:')
        self.get_logger().info('  0 - MoveJ (Joint space PTP motion)')
        self.get_logger().info('  1 - MoveL (Cartesian linear motion)')
        self.get_logger().info('  2 - Open Gripper')
        self.get_logger().info('  3 - Close Gripper')
        self.get_logger().info('==============================================')
    
    def command_callback(self, msg):
        """Handle incoming waypoint commands"""
        for idx, waypoint in enumerate(msg.waypoints): 
            self.get_logger().info(f'Received command - Type: {waypoint}')
        
            if waypoint.type == 0:
                self.move_j(waypoint.pose)
            elif waypoint.type == 1:
                self.move_l(waypoint.pose)
            elif waypoint.type == 2:
                self.open_gripper()
            elif waypoint.type == 3:
                self.close_gripper()
            else:
                self.get_logger().warn(f'Unknown action type: {waypoint.type}')
    
    def move_j(self, target_pose):
        """
        MoveJ: Move to pose using joint space (PTP) motion
        First compute IK, then move to joint configuration
        """
        self.get_logger().info('=' * 50)
        self.get_logger().info('Executing MoveJ (Joint Space PTP Motion)')
        self.get_logger().info('=' * 50)
        
        # Log target
        self.log_pose(target_pose, 'Target')
        
        try:
            # Extract position and orientation
            position = [target_pose.position.x, 
                       target_pose.position.y, 
                       target_pose.position.z]
            quat_xyzw = [target_pose.orientation.x, 
                        target_pose.orientation.y,
                        target_pose.orientation.z, 
                        target_pose.orientation.w]
            
            # Compute IK to get joint configuration
            self.get_logger().info('Computing IK...')
            joint_state = self.moveit2.compute_ik(position, quat_xyzw)
            
            if joint_state is None:
                self.get_logger().error('IK computation failed!')
                return
            
            # Extract just the arm joint positions (first 7 joints)
            joint_positions = joint_state.position[0:7]
            self.get_logger().info(f'IK solution: {[f"{j:.4f}" for j in joint_positions]}')
            
            # Move to joint configuration (PTP motion)
            self.get_logger().info('Executing joint space motion...')
            self.moveit2.move_to_configuration(joint_positions)
            self.moveit2.wait_until_executed()
            
            self.get_logger().info('MoveJ completed successfully!')
            
        except Exception as e:
            self.get_logger().error(f'MoveJ failed: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
        finally:
            self.get_logger().info('=' * 50)
            self.motion_executed = True
    
    def move_l(self, target_pose):
        """
        MoveL: Move to pose using Cartesian linear motion
        cartesian=True uses Cartesian path planning for linear motion
        """
        self.get_logger().info('=' * 50)
        self.get_logger().info('Executing MoveL (Cartesian Linear Motion)')
        self.get_logger().info('=' * 50)
        
        # Log target
        self.log_pose(target_pose, 'Target')
        
        try:
            # Extract position and orientation
            position = [target_pose.position.x, 
                       target_pose.position.y, 
                       target_pose.position.z]
            quat_xyzw = [target_pose.orientation.x, 
                        target_pose.orientation.y,
                        target_pose.orientation.z, 
                        target_pose.orientation.w]
            
            # Move to pose with Cartesian path (linear motion)
            self.get_logger().info('Computing Cartesian path...')
            self.moveit2.move_to_pose(
                position=position,
                quat_xyzw=quat_xyzw,
                cartesian=True  # Cartesian linear motion
            )
            self.moveit2.wait_until_executed()
            
            self.get_logger().info('MoveL completed successfully!')
            
        except Exception as e:
            self.get_logger().error(f'MoveL failed: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
        finally:
            self.get_logger().info('=' * 50)
            self.motion_executed = True
    
    def open_gripper(self):
        """Open the gripper"""
        self.get_logger().info('Opening gripper...')
        # TODO: Implement gripper control
        self.gripper.open()
        self.gripper.wait_until_executed()
        self.get_logger().info("Gripper opened!")
    
    def close_gripper(self):
        """Close the gripper"""
        self.get_logger().info('Closing gripper...')
        # TODO: Implement gripper control
        self.gripper.close()
        self.gripper.wait_until_executed()
        self.get_logger().info("Gripper closed!")
    
    def log_pose(self, pose, label='Pose'):
        """Helper function to log pose information"""
        self.get_logger().info(f'{label}:')
        self.get_logger().info(f'  Position: x={pose.position.x:.4f}, '
                              f'y={pose.position.y:.4f}, '
                              f'z={pose.position.z:.4f}')
        self.get_logger().info(f'  Orientation: x={pose.orientation.x:.4f}, '
                              f'y={pose.orientation.y:.4f}, '
                              f'z={pose.orientation.z:.4f}, '
                              f'w={pose.orientation.w:.4f}')

    def teach_mode_callback(self, msg):
        """Handle teach mode toggle"""
        if msg.data:
            if not self.is_teaching:
                self.get_logger().info('Entering Teach Mode')
                # Start Servo
                if self.servo_start_client.wait_for_service(timeout_sec=1.0):
                    req = Trigger.Request()
                    future = self.servo_start_client.call_async(req)
                    # We don't wait for result to avoid blocking, assuming success
                else:
                    self.get_logger().warn('Servo start service not available')
                
                self.is_teaching = True
        else:
            if self.is_teaching:
                self.get_logger().info('Exiting Teach Mode')
                self.is_teaching = False
                
                # Stop servo motion
                self.servo(linear=[0.0, 0.0, 0.0], angular=[0.0, 0.0, 0.0])

                # Stop Servo Node
                if self.servo_stop_client.wait_for_service(timeout_sec=1.0):
                    req = Trigger.Request()
                    future = self.servo_stop_client.call_async(req)
                else:
                    self.get_logger().warn('Servo stop service not available')
                
                # Move to Home
                self.move_to_home()

    def delta_callback(self, msg):
        """Handle incoming relative movement from VR controller"""
        if not self.is_teaching:
            return

        # Pass command to servo
        # Twist msg has Vector3 linear and Vector3 angular
        self.servo(
            linear=[msg.linear.x, msg.linear.y, msg.linear.z],
            angular=[msg.angular.x, msg.angular.y, msg.angular.z]
        )

    def move_to_home(self):
        """Move to a predefined Home configuration"""
        self.get_logger().info('Moving to Home position...')
        # Define a home configuration (example joint positions)
        # Verify these are safe for FR3!
        home_joints = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785] 
        
        try:
            self.moveit2.move_to_configuration(home_joints)
            self.moveit2.wait_until_executed()
            self.get_logger().info('Reached Home position.')
        except Exception as e:
            self.get_logger().error(f'Failed to move to Home: {e}')


def main():
    rclpy.init()
    
    # Create controller node
    node = FrankaMoveitController()
    
    # Spin the node in background thread(s)
    #executor = rclpy.executors.MultiThreadedExecutor(2)
    #executor.add_node(node)
    #executor_thread = Thread(target=executor.spin, daemon=True, args=())
    #executor_thread.start()
    
    # Wait a bit for initialization
    #node.create_rate(1.0).sleep()
    
    node.get_logger().info('Controller ready and spinning...')
    rclpy.spin(node)
    #try:
    #    if node.exit_after_execution:
            # Wait for motion to be executed, then exit
    #        rate = node.create_rate(10)  # 10 Hz
    #        while not node.motion_executed:
    #            rate.sleep()
            
    #        # Give a moment for completion logging
    #        node.create_rate(2.0).sleep()
            
    #        node.get_logger().info('Motion executed. Exiting...')
   #     else:
            # Keep running indefinitely
   #         executor_thread.join()
            
    #except KeyboardInterrupt:
    #    node.get_logger().info('Keyboard interrupt, shutting down...')
    #finally:
    #    executor.shutdown()
    #    node.destroy_node()
    #    rclpy.shutdown()
    #    exit(0)



if __name__ == '__main__':
    main()
