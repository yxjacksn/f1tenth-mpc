import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import MarkerArray

from mpc_pkg.mpc_core import MPCCore
from mpc_pkg.waypoint_utils import WaypointTracker
from mpc_pkg import viz


def quat_to_yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class MPCNode(Node):
    def __init__(self):
        super().__init__('mpc_node')

        self.declare_parameter('waypoint_csv', '')
        self.declare_parameter('horizon', 10)
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('wheelbase', 0.3302)
        self.declare_parameter('q_x', 10.0)
        self.declare_parameter('q_y', 10.0)
        self.declare_parameter('q_theta', 2.0)
        self.declare_parameter('q_v', 1.0)
        self.declare_parameter('r_delta', 5.0)
        self.declare_parameter('r_a', 1.0)
        self.declare_parameter('r_d_delta', 50.0)
        self.declare_parameter('r_d_a', 1.0)
        self.declare_parameter('delta_max', 0.4189)
        self.declare_parameter('v_min', 0.0)
        self.declare_parameter('v_max', 4.0)
        self.declare_parameter('a_max', 3.0)
        self.declare_parameter('lookahead_idx', 3)
        self.declare_parameter('publish_waypoints', True)
        self.declare_parameter('control_rate', 20.0)

        params = {
            'horizon': self.get_parameter('horizon').value,
            'dt': self.get_parameter('dt').value,
            'wheelbase': self.get_parameter('wheelbase').value,
            'q_x': self.get_parameter('q_x').value,
            'q_y': self.get_parameter('q_y').value,
            'q_theta': self.get_parameter('q_theta').value,
            'q_v': self.get_parameter('q_v').value,
            'r_delta': self.get_parameter('r_delta').value,
            'r_a': self.get_parameter('r_a').value,
            'r_d_delta': self.get_parameter('r_d_delta').value,
            'r_d_a': self.get_parameter('r_d_a').value,
            'delta_max': self.get_parameter('delta_max').value,
            'v_min': self.get_parameter('v_min').value,
            'v_max': self.get_parameter('v_max').value,
            'a_max': self.get_parameter('a_max').value,
        }

        csv_path = self.get_parameter('waypoint_csv').value
        self.lookahead_idx = self.get_parameter('lookahead_idx').value
        rate = self.get_parameter('control_rate').value

        self.get_logger().info('Building MPC solver...')
        self.mpc = MPCCore(params)
        self.get_logger().info('MPC solver ready.')

        self.tracker = WaypointTracker(csv_path)
        self.get_logger().info(f'Loaded {self.tracker.n} waypoints from {csv_path}')

        self._z = None
        self._u_prev = np.zeros(2)

        self._drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self._viz_pub = self.create_publisher(MarkerArray, '/mpc_viz', 10)

        self.create_subscription(Odometry, '/ego_racecar/odom', self._odom_cb, 10)

        if self.get_parameter('publish_waypoints').value:
            self._wp_pub = self.create_publisher(MarkerArray, '/mpc_waypoints', 1)
            self.create_timer(2.0, self._publish_waypoints_once)
            self._wp_published = False

        self.create_timer(1.0 / rate, self._control_loop)
        self._log_count = 0
        self.get_logger().info(f'MPC node running at {rate} Hz')

    def _odom_cb(self, msg: Odometry):
        pos = msg.pose.pose.position
        yaw = quat_to_yaw(msg.pose.pose.orientation)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        speed = math.sqrt(vx ** 2 + vy ** 2)
        self._z = np.array([pos.x, pos.y, yaw, speed])

    def _control_loop(self):
        if self._z is None:
            return

        z = self._z.copy()

        ref, nearest_idx = self.tracker.get_reference(
            z[0], z[1], z[2], z[3],
            self.mpc.N, self.lookahead_idx)

        t0 = time.monotonic()
        u_opt, pred_z, success = self.mpc.solve(z, ref, self._u_prev)
        dt_solve = (time.monotonic() - t0) * 1000

        if not success:
            self.get_logger().warn(f'MPC solve failed ({dt_solve:.1f}ms)')
        else:
            self._u_prev = u_opt.copy()

        target_speed = float(pred_z[1, 3]) if success else float(max(0.0, z[3]))
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = float(u_opt[0])
        drive_msg.drive.speed = target_speed
        self._drive_pub.publish(drive_msg)

        stamp = self.get_clock().now().to_msg()
        ma = MarkerArray()
        ma.markers.extend(viz.make_predicted_trajectory_markers(pred_z, stamp=stamp).markers)
        ma.markers.extend(viz.make_reference_markers(ref, stamp=stamp).markers)
        self._viz_pub.publish(ma)

        self._log_count += 1
        if self._log_count % 20 == 0:
            ref_xy = ref[0, :2]
            car_xy = z[:2]
            dist_to_ref = np.linalg.norm(ref_xy - car_xy)
            self.get_logger().info(
                f'MPC: solve={dt_solve:.1f}ms ok={success} '
                f'δ={u_opt[0]:.3f} a={u_opt[1]:.2f} v={z[3]:.2f} '
                f'wp={nearest_idx}/{self.tracker.n} d_ref={dist_to_ref:.2f}')

    def _publish_waypoints_once(self):
        if self._wp_published:
            return
        stamp = self.get_clock().now().to_msg()
        ma = viz.make_waypoint_markers(self.tracker.waypoints, stamp=stamp)
        self._wp_pub.publish(ma)
        self._wp_published = True
        self.get_logger().info('Published waypoint markers')


def main(args=None):
    rclpy.init(args=args)
    node = MPCNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
