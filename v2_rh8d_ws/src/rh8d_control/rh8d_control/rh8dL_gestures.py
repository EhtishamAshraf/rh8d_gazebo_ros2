"""
                        "Hand Gesture Controller for RH8D (Left) Robotic Hand"
Palm is controlled by a JointTrajectoryController, while fingers are controlled by an EffortController.

palm_trajectory_controller:
      - forearm:1--base:1               --- {Limit: lower="-1.57079632679"  , upper="1.57079632679"}
      - palm_axis:1--forearm:1          --- {Limit: lower="-1.57079632679"  , upper="1.57079632679"}
      - palmL:1--palm_axis:1            --- {Limit: lower="-0.785398163395" , upper="0.785398163395"}
      - Thumb_axis--palmL:1             --- {Limit: lower="0.0",            , upper="1.57079632679"}

finger_effort_controller:
      - Thumb_Methacarpal--Thumb_axis   --- {Limit: lower="0.0"             , upper="1.57079632679"}
      - Index_Proximal--palmL:1         --- {Limit: lower="0.0"             , upper="1.57079632679"}
      - Middle_Proximal--palmL:1        --- {Limit: lower="0.0"             , upper="1.57079632679"}
      - Ring_Proximal--palmL:1          --- {Limit: lower="0.0"             , upper="1.57079632679"}  
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class HandGestureController(Node):
    def __init__(self):
        super().__init__('hand_gesture_controller')

        self.current_joint_states = {}

        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos_profile_sensor_data
        )

        self.palm_trajectory_publisher = self.create_publisher(
            JointTrajectory, '/palm_trajectory_controller/joint_trajectory', 10)

        self.finger_effort_publisher = self.create_publisher(
            Float64MultiArray, '/finger_effort_controller/commands', 10)

        # Sequence of gestures to perform
        self.gestures = [
            {
                "name": "Wrist Rotation (+)",
                "palm": [1.57, 0.0, 0.0, 0.0],
                "fingers": [0.0, 0.0, 0.0, 0.0],
                "checks": [
                    ('forearm:1--base:1', 1.57)
                ]
            },
            {
                "name": "Wrist Rotation (-)",
                "palm": [-1.57, 0.0, 0.0, 0.0],
                "fingers": [0.0, 0.0, 0.0, 0.0],
                "checks": [
                    ('forearm:1--base:1', -1.57)
                ]
            },
            {
                "name": "Wrist Adduction (-)",
                "palm": [-1.57, -1.0, 0.0, 0.0],
                "fingers": [0.0, 0.0, 0.0, 0.0],
                "checks": [
                    ('palm_axis:1--forearm:1', -1.0)
                ]
            },
            {
                "name": "Wrist Adduction (+)",
                "palm": [-1.57, 1.0, 0.0, 0.0],
                "fingers": [0.0, 0.0, 0.0, 0.0],
                "checks": [
                    ('palm_axis:1--forearm:1', 1.0)
                ]
            },
            {
                "name": "Wrist Flexion (+)",
                "palm": [-1.57, 0.0, 0.78, 0.0],
                "fingers": [0.0, 0.0, 0.0, 0.0],
                "checks": [
                    ('palmL:1--palm_axis:1', 0.78)
                ]
            },
            {
                "name": "Wrist Flexion (-)",
                "palm": [-1.57, 0.0, -0.78, 0.0],
                "fingers": [0.0, 0.0, 0.0, 0.0],
                "checks": [
                    ('palmL:1--palm_axis:1', -0.78)
                ]
            },
            {
                "name": "Thumb Adduction",
                "palm": [0.0, 0.0, 0.0, 1.57],
                "fingers": [0.0, 0.0, 0.0, 0.0],
                "checks": [
                    ('Thumb_axis--palmL:1', 1.57)
                ]
            },
            {
                "name": "Hand Close",
                "palm": [0.0, 0.0, 0.0, 1.57],
                "fingers": [0.5, 0.5, 0.5, 0.5],
                "checks": [
                    ('Thumb_Methacarpal--Thumb_axis', 1.57),
                    ('Index_Proximal--palmL:1', 1.57),
                    ('Middle_Proximal--palmL:1', 1.57),
                    ('Ring_Proximal--palmL:1', 1.57)
                ]
            },
            {
                "name": "Victory Sign",
                "palm": [0.0, 0.0, 0.0, 1.57],
                "fingers": [0.5, -0.5, -0.5, 0.5],
                "checks": [
                    ('Thumb_Methacarpal--Thumb_axis', 1.57),
                    ('Index_Proximal--palmL:1', 0.0),
                    ('Middle_Proximal--palmL:1', 0.0),
                    ('Ring_Proximal--palmL:1', 1.57)
                ]
            },
            {
                "name": "Middle Finger Pointing",
                "palm": [0.0, 0.0, 0.0, 1.57],
                "fingers": [0.5, 0.5, -0.5, 0.5],
                "checks": [
                    ('Thumb_Methacarpal--Thumb_axis', 1.57),
                    ('Index_Proximal--palmL:1', 1.57),
                    ('Middle_Proximal--palmL:1', 0.0),
                    ('Ring_Proximal--palmL:1', 1.57)
                ]
            },
            {
                "name": "Index Finger Pointing",
                "palm": [0.0, 0.0, 0.0, 1.57],
                "fingers": [0.5, -0.5, 0.5, 0.5],
                "checks": [
                    ('Thumb_Methacarpal--Thumb_axis', 1.57),
                    ('Index_Proximal--palmL:1', 0.0),
                    ('Middle_Proximal--palmL:1', 1.57),
                    ('Ring_Proximal--palmL:1', 1.57)
                ]
            },
            {
                "name": "Hand Open",
                "palm": [0.0, 0.0, 0.0, 1.57],
                "fingers": [-0.5, -0.5, -0.5, -0.5],
                "checks": [
                    ('Thumb_Methacarpal--Thumb_axis', 0.0),
                    ('Index_Proximal--palmL:1', 0.0),
                    ('Middle_Proximal--palmL:1', 0.0),
                    ('Ring_Proximal--palmL:1', 0.0)
                ]
            }
        ]        

        # State machine
        self.state = 0
        self.state_sent = False

        # Calling the step function at a fixed rate of 20 Hz (0.05 seconds)
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.05, self.step)

        self.get_logger().info("HandGestureController started. Waiting for /joint_states...")

    # Function to update current joint states from the /joint_states topic
    def joint_state_callback(self, msg: JointState):
        for i, name in enumerate(msg.name):
            self.current_joint_states[name] = msg.position[i]
        
    # Function to check if a joint is at the target position within a tolerance
    def is_joint_at_target(self, joint_name: str, target_position: float, tolerance: float = 0.01) -> bool:
        current_position = self.current_joint_states.get(joint_name, None)
        if current_position is None:
            return False
        return abs(current_position - target_position) <= tolerance

    # Function to publish palm trajectory commands
    def publish_palm_trajectory(self, positions):
        position_cmds = JointTrajectory()
        position_cmds.joint_names = [
            'forearm:1--base:1',
            'palm_axis:1--forearm:1',
            'palmL:1--palm_axis:1',
            'Thumb_axis--palmL:1',
        ]
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 1
        position_cmds.points = [point]
        self.palm_trajectory_publisher.publish(position_cmds)

    # Function to publish finger efforts
    def publish_finger_effort(self, efforts):
        effort_cmds = Float64MultiArray()
        effort_cmds.data = efforts
        self.finger_effort_publisher.publish(effort_cmds)

    # Function to execute the gesture sequence based on the current state
    def step(self):

        # waiting for joint states to be populated
        if not self.current_joint_states:
            self.get_logger().info("Waiting for joint states...")
            return

        # 1 second delay at the start to ensure everything is initialized
        if (self.get_clock().now() - self.start_time).nanoseconds < 1e9:
            return

        if self.state >= len(self.gestures):
            self.state = 0  # Loop back to the first gesture

        gesture = self.gestures[self.state]

        if not self.state_sent:
            self.get_logger().info(f"Gesture {self.state}: {gesture['name']}")
            self.publish_palm_trajectory(gesture["palm"])
            self.publish_finger_effort(gesture["fingers"])
            self.state_sent = True

        if all([
            self.is_joint_at_target(joint, target)
            for joint, target in gesture["checks"]
        ]):
            self.get_logger().info("Sequence Completed. Next gesture.")
            self.state += 1
            self.state_sent = False

def main(args=None):
    rclpy.init(args=args)
    node = HandGestureController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()