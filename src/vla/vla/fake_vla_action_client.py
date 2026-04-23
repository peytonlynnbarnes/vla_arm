#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from tf2_ros import Buffer, TransformListener, TransformException


class FakeVLAActionClient(Node):
    """
    Drop-in replacement for vla_action_client.py that fabricates action
    deltas without contacting an OpenVLA server. Publishes the same 7D
    Float32MultiArray ([dx, dy, dz, drx, dry, drz, gripper]) on the same
    topic so the rest of the pipeline (action_to_ee -> vla_ik) is unchanged.

    Translation deltas are interpreted in `base_link` frame (identity
    pass-through in action_to_ee.py), not in the EE local frame.
    """

    def __init__(self):
        super().__init__('fake_vla_action_client')

        self.declare_parameter('action_topic', '/vla_action')
        self.declare_parameter('status_topic', '/openvla/status')
        self.declare_parameter('publish_rate_hz', 1.0)

        # 'sine'     : Lissajous-style oscillating deltas; integrated motion stays bounded
        # 'constant' : emits the constant_* params below every tick
        # 'zero'     : emits all zeros (useful for verifying the IK deadband)
        # 'grab'     : keyframed trajectory to approach, grip, and lift the grab_target_xyz
        self.declare_parameter('mode', 'sine')

        self.declare_parameter('translation_amplitude', 0.02)
        self.declare_parameter('rotation_amplitude', 0.0)
        self.declare_parameter('period_x_sec', 20.0)
        self.declare_parameter('period_y_sec', 30.0)
        self.declare_parameter('period_z_sec', 40.0)

        self.declare_parameter('constant_dx', 0.0)
        self.declare_parameter('constant_dy', 0.0)
        self.declare_parameter('constant_dz', 0.0)
        self.declare_parameter('constant_drx', 0.0)
        self.declare_parameter('constant_dry', 0.0)
        self.declare_parameter('constant_drz', 0.0)

        self.declare_parameter('gripper', 0.0)

        # 'grab' params — target is in base_link frame.
        # Arm spawns at world z=0.2 per arm_gazebo.launch.py, so world(0.45, 0, 0.03)
        # maps to base_link(0.45, 0, -0.17). This is past the 0.255m reach limit;
        # the IK reach clamp will cap how far the arm actually extends.
        self.declare_parameter('grab_target_x', 0.45)
        self.declare_parameter('grab_target_y', 0.0)
        self.declare_parameter('grab_target_z', -0.17)
        self.declare_parameter('grab_hover_height', 0.08)     # meters above target before descent
        self.declare_parameter('grab_lift_height', 0.12)      # meters to lift after gripping
        self.declare_parameter('grab_approach_time_sec', 6.0)
        self.declare_parameter('grab_descend_time_sec', 3.0)
        self.declare_parameter('grab_grip_time_sec', 1.5)
        self.declare_parameter('grab_lift_time_sec', 3.0)
        self.declare_parameter('grab_hold_time_sec', 2.0)     # hover at end before going silent
        self.declare_parameter('grab_base_frame', 'base_link')
        self.declare_parameter('grab_ee_frame', 'gripper_frame_link')
        self.declare_parameter('grab_max_step', 0.04)         # per-tick clamp on delta magnitude
        self.declare_parameter('grab_gripper_open', 0.0)      # jaw angle (rad) during approach/descend
        self.declare_parameter('grab_gripper_closed', 1.0)    # jaw angle (rad) during grip/lift
        # Approach from behind the ball: hover and descend land at target + (offset_x, 0, 0),
        # then an "enclose" phase sweeps forward to the ball with jaws still open before
        # the gripper closes. Avoids front-loading contact that would knock the ball away.
        self.declare_parameter('grab_approach_offset_x', 0.04)
        self.declare_parameter('grab_enclose_time_sec', 1.5)

        self.action_topic = self.get_parameter('action_topic').value
        self.status_topic = self.get_parameter('status_topic').value
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.mode = self.get_parameter('mode').value

        self.translation_amplitude = float(self.get_parameter('translation_amplitude').value)
        self.rotation_amplitude = float(self.get_parameter('rotation_amplitude').value)
        self.period_x = float(self.get_parameter('period_x_sec').value)
        self.period_y = float(self.get_parameter('period_y_sec').value)
        self.period_z = float(self.get_parameter('period_z_sec').value)

        self.constant_dx = float(self.get_parameter('constant_dx').value)
        self.constant_dy = float(self.get_parameter('constant_dy').value)
        self.constant_dz = float(self.get_parameter('constant_dz').value)
        self.constant_drx = float(self.get_parameter('constant_drx').value)
        self.constant_dry = float(self.get_parameter('constant_dry').value)
        self.constant_drz = float(self.get_parameter('constant_drz').value)

        self.gripper_value = float(self.get_parameter('gripper').value)

        target_x = float(self.get_parameter('grab_target_x').value)
        target_y = float(self.get_parameter('grab_target_y').value)
        target_z = float(self.get_parameter('grab_target_z').value)
        hover_h = float(self.get_parameter('grab_hover_height').value)
        lift_h = float(self.get_parameter('grab_lift_height').value)
        t_app = float(self.get_parameter('grab_approach_time_sec').value)
        t_des = float(self.get_parameter('grab_descend_time_sec').value)
        t_grip = float(self.get_parameter('grab_grip_time_sec').value)
        t_lift = float(self.get_parameter('grab_lift_time_sec').value)
        t_hold = float(self.get_parameter('grab_hold_time_sec').value)

        self.grab_base_frame = self.get_parameter('grab_base_frame').value
        self.grab_ee_frame = self.get_parameter('grab_ee_frame').value
        self.grab_max_step = float(self.get_parameter('grab_max_step').value)
        g_open = float(self.get_parameter('grab_gripper_open').value)
        g_close = float(self.get_parameter('grab_gripper_closed').value)
        offset_x = float(self.get_parameter('grab_approach_offset_x').value)
        t_enc = float(self.get_parameter('grab_enclose_time_sec').value)

        # Hover + descend land BEHIND the ball (target_x + offset, further from shoulder).
        # Enclose phase sweeps forward to the ball with jaws still open.
        hover = (target_x + offset_x, target_y, target_z + hover_h)
        behind = (target_x + offset_x, target_y, target_z)
        touch = (target_x, target_y, target_z)
        lift = (target_x, target_y, target_z + lift_h)

        t0 = 0.0
        t1 = t_app
        t2 = t1 + t_des
        t3 = t2 + t_enc
        t4 = t3 + t_grip
        t5 = t4 + t_lift
        t6 = t5 + t_hold

        # Keyframes: (absolute time since start, target_xyz in base_link, gripper).
        # Keyframe 0's xyz is seeded from the current EE pose on the first tick so
        # the approach ramps from wherever the arm is, not from the origin.
        self.keyframes = [
            (t0, None,   g_open),
            (t1, hover,  g_open),
            (t2, behind, g_open),
            (t3, touch,  g_open),
            (t4, touch,  g_close),
            (t5, lift,   g_close),
            (t6, lift,   g_close),
        ]

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._grab_seeded = False
        self._grab_warned = False

        if self.mode not in ('sine', 'constant', 'zero', 'grab'):
            self.get_logger().warn(f"Unknown mode '{self.mode}', falling back to 'zero'")
            self.mode = 'zero'

        if self.publish_rate_hz <= 0.0:
            self.get_logger().warn(f'publish_rate_hz={self.publish_rate_hz} invalid, using 1.0')
            self.publish_rate_hz = 1.0

        self.action_pub = self.create_publisher(Float32MultiArray, self.action_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        self.start_time = self.get_clock().now()
        self.dt = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(self.dt, self.tick)

        self.get_logger().info(
            f"FakeVLA publishing 7D actions to {self.action_topic} "
            f"at {self.publish_rate_hz:.2f} Hz | mode='{self.mode}'"
        )
        if self.mode == 'grab':
            self.get_logger().info(
                f"Grab target (base_link): ({target_x:.3f}, {target_y:.3f}, {target_z:.3f}) | "
                f"hover={hover_h:.3f} lift={lift_h:.3f} | "
                f"total_time={self.keyframes[-1][0]:.1f}s"
            )

    def _lerp(self, a, b, s):
        return a + (b - a) * s

    def _sample_keyframes(self, t: float):
        """Linear interpolation over self.keyframes. Returns (target_xyz, gripper)."""
        if t <= self.keyframes[0][0]:
            return self.keyframes[0][1], self.keyframes[0][2]
        if t >= self.keyframes[-1][0]:
            return self.keyframes[-1][1], self.keyframes[-1][2]

        for i in range(len(self.keyframes) - 1):
            t0, xyz0, g0 = self.keyframes[i]
            t1, xyz1, g1 = self.keyframes[i + 1]
            if t0 <= t <= t1:
                s = (t - t0) / max(t1 - t0, 1e-6)
                xyz = (
                    self._lerp(xyz0[0], xyz1[0], s),
                    self._lerp(xyz0[1], xyz1[1], s),
                    self._lerp(xyz0[2], xyz1[2], s),
                )
                # gripper steps discretely at the start of its segment
                g = g1 if s > 0.0 else g0
                return xyz, g

        return self.keyframes[-1][1], self.keyframes[-1][2]

    def _lookup_ee_pos(self):
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.grab_base_frame, self.grab_ee_frame, rclpy.time.Time()
            )
            return (
                tf_msg.transform.translation.x,
                tf_msg.transform.translation.y,
                tf_msg.transform.translation.z,
            )
        except TransformException:
            return None

    def compute_action(self, t: float):
        if self.mode == 'zero':
            return [0.0] * 6, self.gripper_value

        if self.mode == 'constant':
            deltas = [
                self.constant_dx, self.constant_dy, self.constant_dz,
                self.constant_drx, self.constant_dry, self.constant_drz,
            ]
            return deltas, self.gripper_value

        if self.mode == 'grab':
            # Seed keyframe 0 with the current EE pose so the approach ramps from
            # wherever the arm actually is. Until TF is available, emit zeros.
            if not self._grab_seeded:
                ee = self._lookup_ee_pos()
                if ee is None:
                    if not self._grab_warned:
                        self.get_logger().warn(
                            f"Waiting for TF {self.grab_base_frame} -> {self.grab_ee_frame}..."
                        )
                        self._grab_warned = True
                    return [0.0] * 6, self.keyframes[0][2]
                t0, _, g0 = self.keyframes[0]
                self.keyframes[0] = (t0, ee, g0)
                self._grab_seeded = True
                # Restart the trajectory clock so t=0 aligns with seeding time.
                self.start_time = self.get_clock().now()
                t = 0.0
                self.get_logger().info(
                    f"Grab trajectory seeded at EE=({ee[0]:.3f}, {ee[1]:.3f}, {ee[2]:.3f})"
                )

            target_xyz, gripper = self._sample_keyframes(t)
            cur = self._lookup_ee_pos()
            if cur is None:
                return [0.0] * 6, gripper

            dx = target_xyz[0] - cur[0]
            dy = target_xyz[1] - cur[1]
            dz = target_xyz[2] - cur[2]

            # Clamp per-tick step so any single action respects the max_translation_step
            # that action_to_ee enforces (default 0.05).
            norm = math.sqrt(dx * dx + dy * dy + dz * dz)
            if norm > self.grab_max_step and norm > 1e-9:
                scale = self.grab_max_step / norm
                dx *= scale
                dy *= scale
                dz *= scale
            return [dx, dy, dz, 0.0, 0.0, 0.0], gripper

        # 'sine' — deltas oscillate around zero so integrated motion stays bounded
        ta = self.translation_amplitude
        ra = self.rotation_amplitude
        dx = ta * math.sin(2.0 * math.pi * t / self.period_x)
        dy = ta * math.sin(2.0 * math.pi * t / self.period_y)
        dz = ta * math.sin(2.0 * math.pi * t / self.period_z)
        drx = ra * math.sin(2.0 * math.pi * t / self.period_x)
        dry = ra * math.sin(2.0 * math.pi * t / self.period_y)
        drz = ra * math.sin(2.0 * math.pi * t / self.period_z)
        return [dx, dy, dz, drx, dry, drz], self.gripper_value

    def tick(self):
        t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        deltas, gripper = self.compute_action(t)
        dx, dy, dz, drx, dry, drz = deltas

        msg = Float32MultiArray()
        msg.data = [dx, dy, dz, drx, dry, drz, gripper]
        self.action_pub.publish(msg)

        status = String()
        status.data = (
            f"[FAKE:{self.mode}] t={t:.2f}s | "
            f"dpos=({dx:.5f}, {dy:.5f}, {dz:.5f}) | "
            f"drot=({drx:.5f}, {dry:.5f}, {drz:.5f}) | "
            f"gripper={gripper:.3f}"
        )
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = FakeVLAActionClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
