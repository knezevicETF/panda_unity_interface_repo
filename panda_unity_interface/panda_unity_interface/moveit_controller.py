#!/usr/bin/env python3
from threading import Thread, Lock

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int32
from std_srvs.srv import Trigger

from panda_unity_interface_msgs.msg import Mission, MissionStatus
from pymoveit2 import MoveIt2, GripperInterface, MoveIt2Servo
from pymoveit2.robots import panda as robot


class FrankaMoveitController(Node):
    """
    ROS 2 node to control Franka FR3 robot using MoveIt2 via pymoveit2.

    Action types (Waypoint.type):
      0 - MoveJ (PTP - joint space motion to a pose via IK)
      1 - MoveL (Linear Cartesian motion)
      2 - Open Gripper
      3 - Close Gripper
      4 - JointMove (Joint space motion to joint array)

    Safety zone (Int32 enum on /safety_zone):
      0 - CLEAR   (full speed)
      1 - WARNING (reduced speed)
      2 - DANGER  (stop robot)
    """

    # Safety zone enum
    ZONE_CLEAR = 0
    ZONE_WARNING = 1
    ZONE_DANGER = 2

    def __init__(self):
        super().__init__("franka_moveit_controller")

        # ---- Parameters ----
        self.declare_parameter("joint_names", robot.joint_names())
        self.declare_parameter("base_link_name", robot.base_link_name())
        self.declare_parameter("end_effector_name", robot.end_effector_name())
        self.declare_parameter("group_name", robot.MOVE_GROUP_ARM)

        joint_names = self.get_parameter("joint_names").value
        base_link_name = self.get_parameter("base_link_name").value
        end_effector_name = self.get_parameter("end_effector_name").value
        group_name = self.get_parameter("group_name").value

        callback_group = ReentrantCallbackGroup()

        # ---- State flags ----
        self.is_teaching = False
        self.emergency_stop_active = False
        self.current_zone = self.ZONE_CLEAR

        # Prevent overlapping mission executions (optional but recommended)
        # Prevent overlapping mission executions (optional but recommended)
        # self._mission_lock = Lock()

        # ---- Private Node for Pymoveit2 ----
        # Pymoveit2 internally calls spin_once() which conflicts with the main executor
        # if used on the same node. We create a separate node that is NOT added to
        # the main executor, so Pymoveit2 can manage its spinning safely.
        self._moveit_node = rclpy.create_node("franka_moveit_interface")
        self._gripper_node = rclpy.create_node("franka_moveit_interface_gripepr")
        self._servo_node = rclpy.create_node("franka_moveit_interface_servo")

        # ---- MoveIt2 interfaces ----
        self.moveit2 = MoveIt2(
            node=self._moveit_node,
            joint_names=joint_names,
            base_link_name=base_link_name,
            end_effector_name=end_effector_name,
            group_name=group_name,
            callback_group=callback_group,  
        )

        # Initialize Gripper interface (fixed indentation)
        self.gripper = GripperInterface(
            node=self._gripper_node,
            gripper_joint_names=robot.gripper_joint_names(),
            open_gripper_joint_positions=robot.OPEN_GRIPPER_JOINT_POSITIONS,
            closed_gripper_joint_positions=robot.CLOSED_GRIPPER_JOINT_POSITIONS,
            gripper_group_name=robot.MOVE_GROUP_GRIPPER,
            callback_group=callback_group, 
            gripper_command_action_name="gripper_action_controller/gripper_cmd",
        )

        # Initialize MoveIt2Servo
        # Servo needs continuous callback processing, so it stays on the main node/executor
        self.servo = MoveIt2Servo(
            node=self._servo_node,
            frame_id=base_link_name,
            callback_group=callback_group,
        )

        # ---- Speed scaling ----
        self.default_max_velocity = 0.5
        self.default_max_acceleration = 0.5
        self.moveit2.max_velocity = self.default_max_velocity
        self.moveit2.max_acceleration = self.default_max_acceleration

        # ---- Subscribers ----
        self.delta_subscription = self.create_subscription(
            Twist,
            "vr_controller_delta",
            self.delta_callback,
            10,
            callback_group=callback_group,
        )

        self.teach_mode_subscription = self.create_subscription(
            Bool,
            "teach_mode",
            self.teach_mode_callback,
            10,
            callback_group=callback_group,
        )

        self.move_to_home_subscription = self.create_subscription(
            Bool,
            "move_to_home",
            self.move_to_home_callback,
            10,
            callback_group=callback_group,
        )

        self.subscription = self.create_subscription(
            Mission,
            "mission_command",
            self.command_callback,
            10,
            callback_group=callback_group,
        )

        # Safety zone enum topic
        self.safety_zone_subscription = self.create_subscription(
            Int32,
            "/safety_zone",
            self.safety_zone_callback,
            10,
            callback_group=callback_group,
        )

        self.emergency_stop_subscription = self.create_subscription(
            Bool,
            "/emergency_stop",
            self.emergency_stop_callback,
            10,
            callback_group=callback_group,
        )

        # ---- Publishers ----
        self.mission_execution_publisher = self.create_publisher(
            MissionStatus,
            "mission_execution",
            10,
            callback_group=callback_group,
        )

        # ---- Servo service clients ----
        self.servo_start_client = self.create_client(
            Trigger, "/servo_node/start_servo", callback_group=callback_group
        )
        self.servo_stop_client = self.create_client(
            Trigger, "/servo_node/stop_servo", callback_group=callback_group
        )

        # ---- Startup logs ----
        self.get_logger().info("==============================================")
        self.get_logger().info("Franka MoveIt2 Controller Initialized")
        self.get_logger().info("==============================================")
        self.get_logger().info(f"Group name: {group_name}")
        self.get_logger().info(f"Base link: {base_link_name}")
        self.get_logger().info(f"End effector: {end_effector_name}")
        self.get_logger().info(f"Joint names: {joint_names}")
        self.get_logger().info("==============================================")
        self.get_logger().info("Action types:")
        self.get_logger().info("  0 - MoveJ (Joint space PTP motion)")
        self.get_logger().info("  1 - MoveL (Cartesian linear motion)")
        self.get_logger().info("  2 - Open Gripper")
        self.get_logger().info("  3 - Close Gripper")
        self.get_logger().info("  4 - JointMove (Joint space motion)")
        self.get_logger().info("==============================================")
        self.get_logger().info("Safety zone enum on /safety_zone:")
        self.get_logger().info("  0 - CLEAR, 1 - WARNING, 2 - DANGER")
        self.get_logger().info("==============================================")

    # -------------------------
    # Utilities / Guards
    # -------------------------
    def _can_execute_motion(self) -> bool:
        """Central guard for any motion/gripper execution."""
        if self.emergency_stop_active:
            self.get_logger().warn("E-stop active; refusing to execute motion.")
            return False
        if self.current_zone == self.ZONE_DANGER:
            self.get_logger().warn("Safety zone DANGER; refusing to execute motion.")
            return False
        return True

    def log_joints(self, joints, label="Joints"):
        self.get_logger().info(f"{label}: {[f'{j:.4f}' for j in joints]}")

    def publish_mission_status(self, **kwargs):
        """Helper to publish MissionStatus."""
        msg = MissionStatus()
        msg.movement_started.data = kwargs.get("movement_started", False)
        msg.movement_completed.data = kwargs.get("movement_completed", False)
        msg.movement_failed.data = kwargs.get("movement_failed", False)
        msg.gripper_open_started.data = kwargs.get("gripper_open_started", False)
        msg.gripper_open_completed.data = kwargs.get("gripper_open_completed", False)
        msg.gripper_close_started.data = kwargs.get("gripper_close_started", False)
        msg.gripper_close_completed.data = kwargs.get("gripper_close_completed", False)
        self.mission_execution_publisher.publish(msg)

    def _stop_servo_motion(self):
        """Send zero twist to servo."""
        try:
            self.servo(linear=[0.0, 0.0, 0.0], angular=[0.0, 0.0, 0.0])
        except Exception as e:
            self.get_logger().warn(f"Failed to send zero servo command: {e}")

    def _stop_servo_node(self):
        """Call stop_servo service if available."""
        if self.servo_stop_client.wait_for_service(timeout_sec=1.0):
            req = Trigger.Request()
            self.servo_stop_client.call_async(req)
        else:
            self.get_logger().warn("Servo stop service not available")

    def _start_servo_node(self):
        """Call start_servo service if available."""
        if self.servo_start_client.wait_for_service(timeout_sec=1.0):
            req = Trigger.Request()
            self.servo_start_client.call_async(req)
        else:
            self.get_logger().warn("Servo start service not available")

    # -------------------------
    # Callbacks
    # -------------------------
    def move_to_home_callback(self, msg: Bool):
        if msg.data:
            self.move_to_home()

    def emergency_stop_callback(self, msg: Bool):
        """
        Emergency stop behavior:
          - On trigger: stop MoveIt, stop servo motion, stop servo node, block new commands.
          - On release: clear flag and go home (optional).
        """
        if msg.data:
            if not self.emergency_stop_active:
                self.get_logger().error("Emergency stop triggered!")

            self.emergency_stop_active = True

            # Stop any ongoing MoveIt execution
            try:
                self.moveit2.cancel_execution()
            except Exception as e:
                self.get_logger().warn(f"moveit2.stop() failed: {e}")

            # Stop servo mode too
            self.is_teaching = False
            self._stop_servo_motion()
            self._stop_servo_node()

        else:
            if self.emergency_stop_active:
                self.emergency_stop_active = False
                self.get_logger().info("Emergency stop released! Resetting to home pose.")
                self.move_to_home()

    def safety_zone_callback(self, msg: Int32):
        """
        Handle safety zone enum updates:
          0 CLEAR   -> restore default speed
          1 WARNING -> reduce speed
          2 DANGER  -> stop robot and block new motions
        """
        zone = int(msg.data)
        if zone == self.current_zone:
            return

        self.current_zone = zone

        if zone == self.ZONE_CLEAR:
            if self.moveit2.max_velocity != self.default_max_velocity:
                self.moveit2.max_velocity = self.default_max_velocity
            if self.moveit2.max_acceleration != self.default_max_acceleration:
                self.moveit2.max_acceleration = self.default_max_acceleration
            self.get_logger().info("Safety Zone: CLEAR. Restoring full speed.")

        elif zone == self.ZONE_WARNING:
            reduced_v = 0.1 * self.default_max_velocity
            reduced_a = 0.1 * self.default_max_acceleration
            if self.moveit2.max_velocity != reduced_v:
                self.moveit2.max_velocity = reduced_v
            if self.moveit2.max_acceleration != reduced_a:
                self.moveit2.max_acceleration = reduced_a
            self.get_logger().warn("Safety Zone: WARNING. Reducing speed to 10%.")

        elif zone == self.ZONE_DANGER:
            # Stop current execution immediately
            reduced_v = 0 * self.default_max_velocity
            reduced_a = 0 * self.default_max_acceleration
            if self.moveit2.max_velocity != reduced_v:
                self.moveit2.max_velocity = reduced_v
            if self.moveit2.max_acceleration != reduced_a:
                self.moveit2.max_acceleration = reduced_a
            self.get_logger().error("Safety Zone: DANGER. Stopping robot!")

            # Also stop servo mode
            #self.is_teaching = False
            #self._stop_servo_motion()
            #self._stop_servo_node()

        else:
            self.get_logger().warn(f"Safety Zone: unknown enum value {zone}. Treating as WARNING.")
            reduced_v = 0.1 * self.default_max_velocity
            reduced_a = 0.1 * self.default_max_acceleration
            self.moveit2.max_velocity = reduced_v
            self.moveit2.max_acceleration = reduced_a

    def teach_mode_callback(self, msg: Bool):
        """
        Teach mode:
          - ON: start servo node, allow delta_callback to send twists
          - OFF: stop servo motion, stop servo node, then go home
        """
        if msg.data:
            if not self.is_teaching:
                if not self._can_execute_motion():
                    self.get_logger().warn("Cannot enter teach mode due to safety/E-stop.")
                    return

                self.get_logger().info("Entering Teach Mode")
                self._start_servo_node()
                self.is_teaching = True
        else:
            if self.is_teaching:
                self.get_logger().info("Exiting Teach Mode")
                self.is_teaching = False

                self._stop_servo_motion()
                self._stop_servo_node()

                self.move_to_home()

    def delta_callback(self, msg: Twist):
        """Handle incoming relative movement from VR controller."""
        if not self.is_teaching:
            return
        if not self._can_execute_motion():
            return

        self.servo(
            linear=[msg.linear.x, msg.linear.y, msg.linear.z],
            angular=[msg.angular.x, msg.angular.y, msg.angular.z],
        )

    def command_callback(self, msg: Mission):
        """
        Mission execution:
          - Spawns a thread to avoid blocking the ROS executor.
          - Guarded by _mission_lock.
        """
        # if self._mission_lock.locked():
        #     self.get_logger().warn("Mission already executing; ignoring new command.")
        #     self.publish_mission_status(movement_failed=True)
        #     return

        # Start background thread
        t = Thread(target=self._execute_mission_thread, args=(msg,))
        t.start()

    def _execute_mission_thread(self, msg: Mission):
        """
        Actual blocking execution logic running in a separate thread.
        """
        if not self._can_execute_motion():
            self.publish_mission_status(movement_failed=True)
            return

        # Double-check lock (should typically succeed if callback checked, but race possible)
        # We use a blocking acquire here because we are already in a background thread
        # and we want to ensure exclusive access.
        # if not self._mission_lock.acquire(timeout=1.0):
        #     self.get_logger().warn("Could not acquire mission lock in thread.")
        #     self.publish_mission_status(movement_failed=True)
        #     return

        try:
            for idx, waypoint in enumerate(msg.waypoints):
                # Check guard before every waypoint
                if not self._can_execute_motion():
                    self.get_logger().warn("Aborting mission due to safety/E-stop.")
                    self.publish_mission_status(movement_failed=True)
                    return

                self.get_logger().info(f"Waypoint[{idx}] type={waypoint.type}")

                if waypoint.type == 0:
                    self.publish_mission_status(movement_started=True)
                    ok = self.move_j(waypoint.pose)
                    self.publish_mission_status(
                        movement_started=False,
                        movement_completed=ok,
                        movement_failed=not ok,
                    )

                elif waypoint.type == 1:
                    self.publish_mission_status(movement_started=True)
                    ok = self.move_l(waypoint.pose)
                    self.publish_mission_status(
                        movement_started=False,
                        movement_completed=ok,
                        movement_failed=not ok,
                    )

                elif waypoint.type == 2:
                    self.publish_mission_status(gripper_open_started=True)
                    ok = self.open_gripper()
                    self.publish_mission_status(
                        gripper_open_started=False,
                        gripper_open_completed=ok,
                        movement_failed=not ok,
                    )

                elif waypoint.type == 3:
                    self.publish_mission_status(gripper_close_started=True)
                    ok = self.close_gripper()
                    self.publish_mission_status(
                        gripper_close_started=False,
                        gripper_close_completed=ok,
                        movement_failed=not ok,
                    )

                elif waypoint.type == 4:
                    self.publish_mission_status(movement_started=True)
                    ok = self.joint_move(waypoint.joints)
                    self.publish_mission_status(
                        movement_started=False,
                        movement_completed=ok,
                        movement_failed=not ok,
                    )

                else:
                    self.get_logger().warn(f"Unknown action type: {waypoint.type}")
        except Exception as e:
            self.get_logger().error(f"Mission failed: {e}")
            self.publish_mission_status(movement_failed=True)
        finally:
            # self._mission_lock.release()
            pass

    # -------------------------
    # Motion / Gripper actions
    # -------------------------
    def move_j(self, target_pose) -> bool:
        """MoveJ: compute IK for pose then do joint-space PTP motion."""
        if not self._can_execute_motion():
            return False

        self.get_logger().info("=" * 50)
        self.get_logger().info("Executing MoveJ (Joint Space PTP Motion)")
        self.get_logger().info("=" * 50)

        self.log_pose(target_pose, "Target")

        try:
            position = [
                target_pose.position.x,
                target_pose.position.y,
                target_pose.position.z,
            ]
            quat_xyzw = [
                target_pose.orientation.x,
                target_pose.orientation.y,
                target_pose.orientation.z,
                target_pose.orientation.w,
            ]

            self.get_logger().info("Computing IK...")
            joint_state = self.moveit2.compute_ik(position, quat_xyzw)
            if joint_state is None:
                self.get_logger().error("IK computation failed!")
                return False

            joint_positions = joint_state.position[0:7]
            self.log_joints(joint_positions, "IK solution")

            self.get_logger().info("Executing joint space motion...")
            execution = self.moveit2.move_to_configuration(joint_positions=joint_positions)
            self.moveit2.wait_until_executed()

            self.get_logger().info("MoveJ completed successfully!")
            return True

        except Exception as e:
            self.get_logger().error(f"MoveJ failed: {e}")
            import traceback

            self.get_logger().error(traceback.format_exc())
            return False

        finally:
            self.get_logger().info("=" * 50)

    def move_l(self, target_pose) -> bool:
        """MoveL: Cartesian linear motion to pose."""
        if not self._can_execute_motion():
            return False

        self.get_logger().info("=" * 50)
        self.get_logger().info("Executing MoveL (Cartesian Linear Motion)")
        self.get_logger().info("=" * 50)

        self.log_pose(target_pose, "Target")

        try:
            position = [
                target_pose.position.x,
                target_pose.position.y,
                target_pose.position.z,
            ]
            quat_xyzw = [
                target_pose.orientation.x,
                target_pose.orientation.y,
                target_pose.orientation.z,
                target_pose.orientation.w,
            ]

            self.get_logger().info("Computing Cartesian path...")
            execution = self.moveit2.move_to_pose(
                position=position,
                quat_xyzw=quat_xyzw,
                cartesian=True,
            )
            self.moveit2.wait_until_executed()

            self.get_logger().info("MoveL completed successfully!")
            return True

        except Exception as e:
            self.get_logger().error(f"MoveL failed: {e}")
            import traceback

            self.get_logger().error(traceback.format_exc())
            return False

        finally:
            self.get_logger().info("=" * 50)

    def joint_move(self, target_joints) -> bool:
        """JointMove: PTP motion to given joint array."""
        if not self._can_execute_motion():
            return False

        self.get_logger().info("=" * 50)
        self.get_logger().info("Executing JointMove (Joint Space Motion)")
        self.get_logger().info("=" * 50)

        self.log_joints(target_joints, "Target joints")

        try:
            self.get_logger().info("Computing Joint path...")
            execution = self.moveit2.move_to_configuration(joint_positions=list(target_joints))
            self.moveit2.wait_until_executed()

            self.get_logger().info("JointMove completed successfully!")
            return True

        except Exception as e:
            self.get_logger().error(f"JointMove failed: {e}")
            import traceback

            self.get_logger().error(traceback.format_exc())
            return False

        finally:
            self.get_logger().info("=" * 50)

    def open_gripper(self) -> bool:
        if not self._can_execute_motion():
            return False

        self.get_logger().info("Opening gripper...")
        try:
            self.gripper.open()
            self.gripper.wait_until_executed()
            self.get_logger().info("Gripper opened!")
            return True
        except Exception as e:
            self.get_logger().error(f"Open gripper failed: {e}")
            return False

    def close_gripper(self) -> bool:
        if not self._can_execute_motion():
            return False

        self.get_logger().info("Closing gripper...")
        try:
            self.gripper.close()
            self.gripper.wait_until_executed()
            self.get_logger().info("Gripper closed!")
            return True
        except Exception as e:
            self.get_logger().error(f"Close gripper failed: {e}")
            return False

    def log_pose(self, pose, label="Pose"):
        self.get_logger().info(f"{label}:")
        self.get_logger().info(
            f"  Position: x={pose.position.x:.4f}, y={pose.position.y:.4f}, z={pose.position.z:.4f}"
        )
        self.get_logger().info(
            "  Orientation: "
            f"x={pose.orientation.x:.4f}, y={pose.orientation.y:.4f}, "
            f"z={pose.orientation.z:.4f}, w={pose.orientation.w:.4f}"
        )

    def move_to_home(self):
        """
        Public method to trigger Move to Home in a separate thread.
        This ensures we don't block the executor.
        """
        # if self._mission_lock.locked():
        #     self.get_logger().warn("System busy (lock held); cannot move to home.")
        #     return

        t = Thread(target=self._move_to_home_thread)
        t.start()

    def _move_to_home_thread(self):
        """Thread worker for moving to home."""
        # Double-check safety
        if self.emergency_stop_active:
            self.get_logger().warn("E-stop active; refusing to move home thread.")
            return

        # if not self._mission_lock.acquire(timeout=1.0):
        #     self.get_logger().warn("Could not acquire lock for move_to_home.")
        #     return

        try:
            self._move_to_home_impl()
        finally:
            # self._mission_lock.release()
            pass

    def _move_to_home_impl(self):
        """Move to a predefined Home configuration (blocking implementation)."""
        if self.emergency_stop_active:
            self.get_logger().warn("E-stop active; refusing to move home.")
            return
        if self.current_zone == self.ZONE_DANGER:
            self.get_logger().warn("Safety zone DANGER; refusing to move home.")
            return

        self.get_logger().info("Moving to Home position...")
        home_joints = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

        try:
            self.moveit2.move_to_configuration(joint_positions=home_joints)
            self.moveit2.wait_until_executed()
            self.get_logger().info("Reached Home position.")
        except Exception as e:
            self.get_logger().error(f"Failed to move to Home: {e}")


def main():
    rclpy.init()
    node = FrankaMoveitController()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.add_node(node._moveit_node)
    executor.add_node(node._gripper_node)
    executor.add_node(node._servo_node)

    node.get_logger().info("Controller ready and spinning...")
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down...")
    finally:
        executor.shutdown()
        node._moveit_node.destroy_node() 
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
