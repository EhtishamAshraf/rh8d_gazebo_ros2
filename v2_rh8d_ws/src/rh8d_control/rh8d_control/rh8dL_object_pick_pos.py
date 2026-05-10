"""
Object picking using RH8D hand {Position controlled version}

All commanded joints are controlled by a single JointTrajectoryController.

joint_trajectory_controller joints:
    - forearm:1--base:1
    - palm_axis:1--forearm:1
    - palmL:1--palm_axis:1
    - Thumb_axis--palmL:1
    - Thumb_Methacarpal--Thumb_axis
    - Index_Proximal--palmL:1
    - Middle_Proximal--palmL:1
    - Ring_Proximal--palmL:1

Note:
    For Coke can, the finger joints are given only 0.7 rad of flexion, otherwise the fingers push the object, instead of grasping it. 
    This is likely due to the lack of force control and/or the fact that the simulated object is very light. 
    In a real robot, we would close the fingers fully and use force sensors and/or tactile sensors to detect contact, rather than relying on position thresholds as in this simulation.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException

class HandGestureController(Node):
    def __init__(self):
        super().__init__('rh8d_object_pick')

        self.current_joint_states = {}
        self.current_joint_velocities = {}
        self.have_joint_states = False

        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos_profile_sensor_data
        )

        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        self.controlled_joints = [
            'forearm:1--base:1',
            'palm_axis:1--forearm:1',
            'palmL:1--palm_axis:1',
            'Thumb_axis--palmL:1',
            'Thumb_Methacarpal--Thumb_axis',
            'Index_Proximal--palmL:1',
            'Middle_Proximal--palmL:1',
            'Ring_Proximal--palmL:1',
        ]

        # Joint limits for the "close" position of each joint
        self.close_limits = {
            'Thumb_Methacarpal--Thumb_axis': 1.57,
            'Index_Proximal--palmL:1': 1.57,
            'Middle_Proximal--palmL:1': 1.57,
            'Ring_Proximal--palmL:1': 1.57,
        }

        # Stable-contact detection state
        self.contact_counter = {}
        self.required_contact_cycles = 5    # Number of consecutive cycles with contact to confirm a stable grasp
        
        # For Coke can:
        # self.thumb_flexion = 1.57
        # self.finger_flexion = 0.7         

        # For ball: In gazebo, drag the ball a bit towards +x-axis (red_arrow) to make it easier to grasp so that the ball is grabed by thumb, index, middle *or/and* ring fingers.
        self.thumb_flexion = 1.57
        self.finger_flexion = 0.75          # Ball (more slippery, needs more flexion to maintain contact)

        # Sequence of gestures to perform
        self.gestures = [
            {
                "name": "Hand Open with Thumb Adduction",
                "positions": [0.0, 0.0, 0.0, 1.57, 0.0, 0.0, 0.0, 0.0],
                "checks": [
                    ('Thumb_axis--palmL:1', 1.57),
                ]
            },
            {
                "name": "Hand Close",
                "positions": [0.0, 0.0, 0.0, 1.57, self.thumb_flexion, self.finger_flexion, self.finger_flexion, self.finger_flexion],
                "use_contact": True
            },
            {
                "name": "Wrist Rotation (-)",
                "positions": [-1.57, 0.0, 0.0, 1.57, self.thumb_flexion, self.finger_flexion, self.finger_flexion, self.finger_flexion],
                "checks": [
                    ('forearm:1--base:1', -1.57),
                ]
            },
            {
                "name": "Wrist Adduction (-)",
                "positions": [-1.57, -0.8, 0.0, 1.57, self.thumb_flexion, self.finger_flexion, self.finger_flexion, self.finger_flexion],
                "checks": [
                    ('palm_axis:1--forearm:1', -0.8),
                ]
            },
            {
                "name": "Wrist Adduction (+)",
                "positions": [-1.57, 0.8, 0.0, 1.57, self.thumb_flexion, self.finger_flexion, self.finger_flexion, self.finger_flexion],
                "checks": [
                    ('palm_axis:1--forearm:1', 0.8),
                ]
            },
            {
                "name": "Wrist Flexion (+)",
                "positions": [-1.57, 0.8, 0.78, 1.57, self.thumb_flexion, self.finger_flexion, self.finger_flexion, self.finger_flexion],
                "checks": [
                    ('palmL:1--palm_axis:1', 0.78),
                ]
            },
            {
                "name": "Wrist Flexion (-)",
                "positions": [-1.57, 0.8, -0.78, 1.57, self.thumb_flexion, self.finger_flexion, self.finger_flexion, self.finger_flexion],
                "checks": [
                    ('palmL:1--palm_axis:1', -0.78),
                ]
            },
            {
                "name": "Wrist Rotation (+)",
                "positions": [1.57, 0.0, 0.0, 1.57, self.thumb_flexion, self.finger_flexion, self.finger_flexion, self.finger_flexion],
                "checks": [
                    ('forearm:1--base:1', 1.57),
                ]
            },
            {
                "name": "Hand Open",
                "positions": [1.57, 0.0, 0.0, 1.57, 0.0, 0.0, 0.0, 0.0],
                "checks": [
                    ('Thumb_Methacarpal--Thumb_axis', 0.0),
                    ('Index_Proximal--palmL:1', 0.0),
                    ('Middle_Proximal--palmL:1', 0.0),
                    ('Ring_Proximal--palmL:1', 0.0),
                ]
            }
        ]

        # State machine
        self.state = 0
        self.state_sent = False
        self.start_time = self.get_clock().now()

        # Calling the step function at a fixed rate of 20 Hz (0.05 seconds)
        self.timer = self.create_timer(0.05, self.step)

        self.get_logger().info("HandGestureController started...")

    # Function to update current joint states from the /joint_states topic
    def joint_state_callback(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_states[name] = msg.position[i]
            if i < len(msg.velocity):
                self.current_joint_velocities[name] = msg.velocity[i]
        self.have_joint_states = True

    # Function to check if a joint is at the target position within a tolerance
    def is_joint_at_target(self, joint_name: str, target_position: float, tolerance: float = 0.03) -> bool:
        current_position = self.current_joint_states.get(joint_name, None)
        if current_position is None:
            return False
        diff = abs(current_position - target_position)
        print(f"difference for {joint_name}: {diff:.4f} (current: {current_position:.4f}, target: {target_position:.4f})")
        return abs(current_position - target_position) <= tolerance

    # Function to detect contact based on joint velocity and position relative to limits
    def is_grasp_contact(self, joint_name: str,
                         velocity_threshold: float = 0.025,
                         limit_margin: float = 0.08) -> bool:

        position = self.current_joint_states.get(joint_name, None)
        velocity = self.current_joint_velocities.get(joint_name, None)
        close_limit = self.close_limits.get(joint_name, 1.57)

        if position is None or velocity is None:
            return False

        velocity_small = abs(velocity) < velocity_threshold
        not_at_limit = position < (close_limit - limit_margin)

        print(f"Contact check for {joint_name}: position={position:.4f}, velocity={velocity:.4f}, not_at_limit={not_at_limit}, velocity_small={velocity_small}")

        return velocity_small and not_at_limit

    # Function to require stable contact detection over multiple cycles
    def is_grasp_contact_stable(self, joint_name: str) -> bool:

        contact_now = self.is_grasp_contact(joint_name)

        if joint_name not in self.contact_counter:
            self.contact_counter[joint_name] = 0

        if contact_now:
            self.contact_counter[joint_name] += 1
        else:
            self.contact_counter[joint_name] = 0

        return self.contact_counter[joint_name] >= self.required_contact_cycles

    # Function to publish position trajectory commands for all controlled joints
    def publish_full_trajectory(self, positions, duration_sec=1):
        position_cmds = JointTrajectory()
        position_cmds.joint_names = self.controlled_joints

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = duration_sec

        position_cmds.points = [point]
        self.trajectory_publisher.publish(position_cmds)

    # Function to execute the gesture sequence based on the current state
    def step(self):
        
        # waiting for joint states to be populated        
        if not self.have_joint_states:
            return

        # 1 second delay at the start to ensure everything is initialized
        if (self.get_clock().now() - self.start_time).nanoseconds < 1e9:
            return

        if self.state >= len(self.gestures):
            self.get_logger().info("All gestures completed.")
            if rclpy.ok():
                rclpy.shutdown()
            return

        gesture = self.gestures[self.state]

        if not self.state_sent:
            self.get_logger().info(f"Gesture {self.state}: {gesture['name']}")
            self.publish_full_trajectory(gesture["positions"], duration_sec=1)
            self.state_sent = True
            self.contact_counter = {}

        # Grip objects based on contact detection instead of just joint positions
        if gesture.get("use_contact", False):
            done = all([
                self.is_grasp_contact_stable('Thumb_Methacarpal--Thumb_axis'),
                # self.is_grasp_contact_stable('Index_Proximal--palmL:1'),
                self.is_grasp_contact_stable('Middle_Proximal--palmL:1'),
                self.is_grasp_contact_stable('Ring_Proximal--palmL:1'),
            ])
        else:
            done = all(
                self.is_joint_at_target(joint, target)
                for joint, target in gesture["checks"]
            )

        # If the gesture is done, move to the next one
        if done:
            self.get_logger().info(f"{gesture['name']} complete.")
            self.state += 1
            self.state_sent = False


def main(args=None):
    rclpy.init(args=args)
    node = HandGestureController()
    ex = SingleThreadedExecutor()
    ex.add_node(node)

    try:
        while rclpy.ok():
            ex.spin_once(timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            ex.remove_node(node)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
