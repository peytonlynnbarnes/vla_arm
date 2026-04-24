#!/usr/bin/env python3
"""Tk slider GUI for driving the SO-101 arm + gripper controllers live.

Each slider publishes a short JointTrajectory (current_value -> slider_value
over 0.4 s) to the matching controller the moment you release / step the
slider. Initial positions are read from /joint_states.
"""
import threading
import tkinter as tk

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
ARM_LIMITS = {
    'shoulder_pan':  (-1.91, 1.91),
    'shoulder_lift': (-1.75, 1.75),
    'elbow_flex':    (-1.69, 1.69),
    'wrist_flex':    (-1.66, 1.66),
    'wrist_roll':    (-2.79, 2.79),
}
GRIPPER_LIMITS = (-0.17, 1.74)


class SliderNode(Node):
    def __init__(self):
        super().__init__('joint_sliders')
        self.arm_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.grip_pub = self.create_publisher(
            JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        self.latest = {}
        self.create_subscription(
            JointState, '/joint_states', self._on_js, 10)

    def _on_js(self, msg: JointState):
        for n, p in zip(msg.name, msg.position):
            self.latest[n] = p

    def publish_arm(self, positions):
        tj = JointTrajectory()
        tj.joint_names = ARM_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in positions]
        pt.time_from_start.sec = 0
        pt.time_from_start.nanosec = 400_000_000
        tj.points.append(pt)
        self.arm_pub.publish(tj)

    def publish_gripper(self, value):
        tj = JointTrajectory()
        tj.joint_names = ['gripper']
        pt = JointTrajectoryPoint()
        pt.positions = [float(value)]
        pt.time_from_start.sec = 0
        pt.time_from_start.nanosec = 400_000_000
        tj.points.append(pt)
        self.grip_pub.publish(tj)


def main():
    rclpy.init()
    node = SliderNode()

    spin_thread = threading.Thread(
        target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()

    # Wait briefly for joint_states so sliders start at current pose.
    import time
    for _ in range(50):
        if all(j in node.latest for j in ARM_JOINTS + ['gripper']):
            break
        time.sleep(0.1)

    root = tk.Tk()
    root.title('SO-101 joint sliders')

    arm_vars = {}

    def on_arm_change(_=None):
        positions = [arm_vars[j].get() for j in ARM_JOINTS]
        node.publish_arm(positions)

    for j in ARM_JOINTS:
        lo, hi = ARM_LIMITS[j]
        frame = tk.Frame(root); frame.pack(fill='x', padx=6, pady=2)
        tk.Label(frame, text=j, width=14, anchor='w').pack(side='left')
        v = tk.DoubleVar(value=node.latest.get(j, 0.0))
        arm_vars[j] = v
        s = tk.Scale(frame, from_=lo, to=hi, resolution=0.01, orient='horizontal',
                     variable=v, length=380, command=on_arm_change)
        s.pack(side='left', fill='x', expand=True)

    grip_frame = tk.Frame(root); grip_frame.pack(fill='x', padx=6, pady=2)
    tk.Label(grip_frame, text='gripper', width=14, anchor='w').pack(side='left')
    grip_var = tk.DoubleVar(value=node.latest.get('gripper', 0.0))
    def on_grip_change(_=None):
        node.publish_gripper(grip_var.get())
    tk.Scale(grip_frame, from_=GRIPPER_LIMITS[0], to=GRIPPER_LIMITS[1],
             resolution=0.01, orient='horizontal',
             variable=grip_var, length=380, command=on_grip_change).pack(
        side='left', fill='x', expand=True)

    def home():
        for j in ARM_JOINTS:
            arm_vars[j].set(0.0)
        on_arm_change()
    tk.Button(root, text='HOME (zeros)', command=home).pack(pady=4)

    def on_close():
        root.destroy()
        rclpy.shutdown()
    root.protocol('WM_DELETE_WINDOW', on_close)
    root.mainloop()


if __name__ == '__main__':
    main()
