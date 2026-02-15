
SCRIPT_HEADER = """#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pymoveit2 import MoveIt2, GripperInterface
from pymoveit2.robots import panda as robot
import time

def main():
    rclpy.init()
    
    # Initialize Node
    node = Node('generated_program_executor')
    
    # Initialize MoveIt2
    moveit2 = MoveIt2(
        node=node,
        joint_names=robot.joint_names(),
        base_link_name=robot.base_link_name(),
        end_effector_name=robot.end_effector_name(),
        group_name=robot.MOVE_GROUP_ARM
    )
    
    # Initialize Gripper
    gripper = GripperInterface(
        node=node,
        gripper_joint_names=robot.gripper_joint_names(),
        open_gripper_joint_positions=robot.OPEN_GRIPPER_JOINT_POSITIONS,
        closed_gripper_joint_positions=robot.CLOSED_GRIPPER_JOINT_POSITIONS,
        gripper_group_name=robot.MOVE_GROUP_GRIPPER,
        gripper_command_action_name="gripper_action_controller/gripper_cmd",
    )
    
    # Wait for initialization
    node.get_logger().info("Waiting for initialization...")
    time.sleep(1.0)
    
    try:
"""

SCRIPT_FOOTER = """
        node.get_logger().info("Program completed successfully!")
        
    except Exception as e:
        node.get_logger().error(f"Program execution failed: {e}")
        import traceback
        node.get_logger().error(traceback.format_exc())
        
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
"""

MOVE_J_TEMPLATE = """
        # MoveJ Action
        node.get_logger().info("Executing MoveJ...")
        position = [{px}, {py}, {pz}]
        quat_xyzw = [{qx}, {qy}, {qz}, {qw}]
        
        joint_state = moveit2.compute_ik(position, quat_xyzw)
        if joint_state:
            moveit2.move_to_configuration(joint_state.position[0:7])
            moveit2.wait_until_executed()
        else:
            raise Exception("IK failed for MoveJ target")
"""

MOVE_L_TEMPLATE = """
        # MoveL Action
        node.get_logger().info("Executing MoveL...")
        position = [{px}, {py}, {pz}]
        quat_xyzw = [{qx}, {qy}, {qz}, {qw}]
        
        moveit2.move_to_pose(
            position=position,
            quat_xyzw=quat_xyzw,
            cartesian=True
        )
        moveit2.wait_until_executed()
"""

OPEN_GRIPPER_TEMPLATE = """
        # Open Gripper
        node.get_logger().info("Opening Gripper...")
        gripper.open()
        gripper.wait_until_executed()
"""

CLOSE_GRIPPER_TEMPLATE = """
        # Close Gripper
        node.get_logger().info("Closing Gripper...")
        gripper.close()
        gripper.wait_until_executed()
"""

JOINT_MOVE_TEMPLATE = """
        # Joint Move Action
        node.get_logger().info("Executing Joint Move...")
        joint_positions = [{j0}, {j1}, {j2}, {j3}, {j4}, {j5}, {j6}]
        
        moveit2.move_to_configuration(joint_positions=joint_positions)
        moveit2.wait_until_executed()
"""
