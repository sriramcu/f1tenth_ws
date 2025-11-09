#!/usr/bin/env python3
import os, yaml, math, rclpy, threading, time
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt

def yaw_from_quat(x,y,z,w):
    # ZYX -> yaw
    s = 2*(w*z + x*y)
    c = 1 - 2*(y*y + z*z)
    return math.atan2(s, c)

class FakeSLAM(Node):
    def __init__(self):
        super().__init__('mvp_fake_slam')
        self.declare_parameter('gt_map_yaml', '')     # e.g. <share_of_f1tenth_gym_ros>/maps/levine.yaml
        self.declare_parameter('lap_seconds', 20.0)    # stop/plot after this many seconds
        self.declare_parameter('odom_topic', 'ego_racecar/odom')
        self.declare_parameter('scan_topic', 'scan')

        self.odom = None
        self.points = []  # global XY points accumulated

        self.sub_o = self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self.cb_odom, 10)
        self.sub_s = self.create_subscription(LaserScan, self.get_parameter('scan_topic').value, self.cb_scan, 10)

        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        self.timer = self.create_timer(0.5, self.check_done)

    def cb_odom(self, msg):
        self.odom = msg

    def cb_scan(self, msg: LaserScan):
        if self.odom is None: return
        p = self.odom.pose.pose.position
        q = self.odom.pose.pose.orientation
        yaw = yaw_from_quat(q.x,q.y,q.z,q.w)
        x, y = p.x, p.y

        angles = msg.angle_min + np.arange(len(msg.ranges))*msg.angle_increment
        ranges = np.array(msg.ranges, dtype=np.float32)
        # quick validity mask
        mask = np.isfinite(ranges) & (ranges > 0.05) & (ranges < msg.range_max)
        angles = angles[mask]; ranges = ranges[mask]

        # points in base frame
        xs = ranges*np.cos(angles); ys = ranges*np.sin(angles)
        # transform to global with odom (assume lidar at base_link origin)
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        Xg = x + cos_y*xs - sin_y*ys
        Yg = y + sin_y*xs + cos_y*ys
        self.points.extend(zip(Xg.tolist(), Yg.tolist()))

    def check_done(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if (now - self.start_time) >= float(self.get_parameter('lap_seconds').value):
            self.timer.cancel()
            threading.Thread(target=self.plot_and_exit, daemon=True).start()

    def plot_and_exit(self):
        gt_yaml = self.get_parameter('gt_map_yaml').value
        fig, ax = plt.subplots(figsize=(7,7))
        # draw ground-truth map if provided
        if gt_yaml and os.path.exists(gt_yaml):
            with open(gt_yaml, 'r') as f:
                info = yaml.safe_load(f)
            img_path = os.path.join(os.path.dirname(gt_yaml), info['image'])
            img = plt.imread(img_path)
            res = float(info['resolution'])
            origin = info['origin']  # [x,y,theta]
            # map image is in pixels; convert to world coords
            h, w = img.shape[:2]
            extent = [origin[0], origin[0]+w*res, origin[1], origin[1]+h*res]
            ax.imshow(np.flipud(img), extent=extent, cmap='Greens', alpha=0.5, origin='lower')
        # plot accumulated “SLAM” points
        if self.points:
            P = np.array(self.points)
            ax.scatter(P[:,0], P[:,1], s=1, c='r', label='Accumulated LiDAR (fake SLAM)')
        ax.set_aspect('equal', 'box')
        ax.set_title('Ground Truth (green) vs Accumulated LiDAR (red)')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best')
        out = os.path.expanduser('~/mvp_fake_slam_overlay.png')
        plt.savefig(out, dpi=150)
        print(f'[MVP] Saved overlay to: {out}')
        plt.show()
        rclpy.shutdown()

def main():
    rclpy.init()
    node = FakeSLAM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
