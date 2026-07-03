#!/usr/bin/env python3

import math
import time
import json
import os
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    qos_profile_sensor_data,
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


ST_IDLE = "IDLE"
ST_FOLLOW = "FOLLOWING"
ST_AVOID = "AVOIDING"
ST_REACQ = "REACQUIRE"

WAYPOINTS_FILE = os.path.expanduser("~/waypoints.json")

DEFAULT_WP = {
    "출입문": [2.0, 0.0],
    "입국문": [2.0, 1.5],
    "의자": [0.5, 1.0],
    "책상": [1.5, 1.5],
    "home": [0.0, 0.0],
}

MODEL_PATHS = [
    "/home/ns2022427138/yolov8s.pt",
    "/home/ns2022427138/yolov8n.pt",
    "yolov8n.pt",
]


class HumanFollowNode(Node):
    def __init__(self):
        super().__init__("human_follow_yolo_lidar")

        self.bridge = CvBridge()
        model_path = next((p for p in MODEL_PATHS if os.path.exists(p)), "yolov8n.pt")
        self.model = YOLO(model_path)
        self.get_logger().info(f"YOLO 모델: {model_path}")

        # First inference is slow. Warm up once.
        dummy = np.zeros((320, 320, 3), dtype=np.uint8)
        self.model(dummy, verbose=False, imgsz=320)
        self.get_logger().info("YOLO 워밍업 완료")

        self.scan_msg = None
        self.camera_fov_deg = 62.0


        self.target_dist = 1.0
        self.follow_dead_low = 0.90
        self.follow_dead_high = 1.10


        self.too_close = 0.75
        self.person_backoff_dist = 0.55


        self.max_v = 0.08
        self.max_w = 0.25
        self.k_v = 0.30
        self.k_w = 0.28

        self.angle_dead = 12.0       
        self.rotate_only = 22.0     


        self.conf_thresh = 0.25


        self.front_fov = 42.0
        self.obs_enter = 0.68
        self.obs_clear = 0.85
        self.critical = 0.30


        self.exc_margin = 0.25


        self.safe_side_dist = 0.60

        self.avoid_turn_w = 0.15
        self.avoid_arc_v = 0.05
        self.avoid_arc_w = 0.08

        # After front becomes clear, keep moving forward briefly.
        # This prevents "almost avoided but stopped too early".
        self.avoid_push_steps = 0
        self.avoid_push_steps_max = 18
        self.avoid_push_v = 0.045
        self.avoid_push_w = 0.00

        self._avoid_dir_locked = False
        self._avoid_dwell = 0
        self.avoid_dwell_min = 12
        self.avoid_dir = 1.0


        self.grace_sec = 0.4
        self.search_sec = 5.0
        self.search_w_fast = 0.18
        self.search_w_slow = 0.14
        self.search_timeout = 80.0
        self._search_dir = 1.0

        self._reacq_confirm = 0
        self.reacq_confirm_need = 1
        self._last_seen_before = 0.0


        self._dist_hist = deque(maxlen=3)
        self._front_hist = deque(maxlen=2)


        self.state = ST_IDLE
        self.last_frame = None
        self.last_best = None

        self.registered = False
        self.follow_on = False
        self.target_hist = None

        self.last_seen = 0.0
        self.last_p_ang = 0.0
        self.last_p_dist = None
        self.last_turn_dir = 1.0

        self.prev_v = 0.0
        self.prev_w = 0.0
        self.smooth_a = 0.40


        self.wp = DEFAULT_WP.copy()
        if os.path.exists(WAYPOINTS_FILE):
            try:
                with open(WAYPOINTS_FILE, "r") as f:
                    self.wp.update(json.load(f))
            except Exception:
                pass

        self.nav2_ok = False
        self._nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.create_timer(2.0, self._chk_nav2)

        cmd_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.create_subscription(Image, "/camera/image_raw", self.image_cb, 10)
        self.create_subscription(LaserScan, "/scan", self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(String, "/yolo_command", self.cmd_cb, cmd_qos)

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.det_pub = self.create_publisher(String, "/yolo_detections", 10)

        self.get_logger().info("═══════════════════════════════════════")
        self.get_logger().info("  Human Follow — Integrated Stable Build Ready")
        self.get_logger().info("  명령: register / follow / stop / reset")
        self.get_logger().info("═══════════════════════════════════════")



    def _chk_nav2(self):
        if self._nav.wait_for_server(timeout_sec=0.1) and not self.nav2_ok:
            self.nav2_ok = True
            self.get_logger().info("Nav2 연결 완료")

    def _goto(self, x, y, label=""):
        if not self.nav2_ok:
            self.get_logger().warn("Nav2 미연결")
            return

        self.follow_on = False
        self._set(ST_IDLE)
        self.publish_stop()

        g = PoseStamped()
        g.header.stamp = self.get_clock().now().to_msg()
        g.header.frame_id = "map"
        g.pose.position.x = float(x)
        g.pose.position.y = float(y)
        g.pose.orientation.w = 1.0

        self.goal_pub.publish(g)
        self.get_logger().info(f"[NAV2] → {label or f'({x:.2f},{y:.2f})'}")



    def scan_cb(self, msg):
        self.scan_msg = msg

    def cmd_cb(self, msg):
        c = msg.data.strip().lower()

        if c == "register":
            if self.last_frame is None or self.last_best is None:
                self.get_logger().warn("등록 실패 — 카메라에 사람이 없습니다.")
                return

            self.target_hist = self.person_hist(self.last_frame, self.last_best)
            self.registered = True
            self.follow_on = False

            self._set(ST_IDLE)
            self.publish_stop()
            self.get_logger().info("✓ 등록 완료 — 'follow' 명령으로 추적 시작")

        elif c == "follow":
            if not self.registered:
                self.get_logger().warn("먼저 'register' 명령을 실행하세요.")
                return

            self.follow_on = True
            self.last_seen = time.time()

            self.prev_v = 0.0
            self.prev_w = 0.0
            self._dist_hist.clear()
            self._front_hist.clear()

            self._avoid_dir_locked = False
            self._avoid_dwell = 0
            self.avoid_push_steps = 0

            self._reacq_confirm = 0
            self._search_dir = self.last_turn_dir

            self._set(ST_FOLLOW)
            self.get_logger().info("▶ 추적 시작")

        elif c == "stop":
            self.follow_on = False
            self._set(ST_IDLE)
            self.publish_stop()
            self.get_logger().info("■ 정지")

        elif c == "reset":
            self.registered = False
            self.follow_on = False
            self.target_hist = None
            self.last_best = None
            self.last_frame = None
            self.last_p_dist = None

            self.prev_v = 0.0
            self.prev_w = 0.0
            self._dist_hist.clear()
            self._front_hist.clear()

            self._avoid_dir_locked = False
            self._avoid_dwell = 0
            self.avoid_push_steps = 0
            self._reacq_confirm = 0
            self._search_dir = self.last_turn_dir

            self._set(ST_IDLE)
            self.publish_stop()
            self.get_logger().info("↺ 초기화 완료")

        elif c.startswith("goto:"):
            t = c.split(":", 1)[1].strip()
            if t in self.wp:
                x, y = self.wp[t]
                self._goto(x, y, t)
            else:
                try:
                    x, y = [float(v) for v in t.split(",")]
                    self._goto(x, y)
                except Exception:
                    self.get_logger().warn(f"goto 오류 — 알려진 장소: {list(self.wp.keys())}")

        elif c.startswith("setwp:"):
            try:
                _, n, xy = c.split(":", 2)
                x, y = [float(v) for v in xy.split(",")]
                self.wp[n.strip()] = [x, y]
                with open(WAYPOINTS_FILE, "w") as f:
                    json.dump(self.wp, f, ensure_ascii=False, indent=2)
                self.get_logger().info(f"웨이포인트 저장: {n}=({x:.2f},{y:.2f})")
            except Exception as e:
                self.get_logger().warn(f"setwp 오류: {e}")


    def _set(self, s):
        if self.state != s:
            self.get_logger().info(f"[FSM] {self.state} → {s}")
            self.prev_v = 0.0
            self.prev_w = 0.0
            self.state = s

    def norm(self, a):
        if isinstance(a, np.ndarray):
            return np.arctan2(np.sin(a), np.cos(a))
        return math.atan2(math.sin(float(a)), math.cos(float(a)))

    def clamp(self, v, lo, hi):
        return max(min(v, hi), lo)

    def smooth(self, tv, tw):
        v = self.smooth_a * tv + (1.0 - self.smooth_a) * self.prev_v
        w = self.smooth_a * tw + (1.0 - self.smooth_a) * self.prev_w

        self.prev_v = v
        self.prev_w = w

        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(w)
        return t

    def to_bgr(self, msg):
        e = msg.encoding.lower()

        if e == "rgb8":
            return cv2.cvtColor(self.bridge.imgmsg_to_cv2(msg, "rgb8"), cv2.COLOR_RGB2BGR)

        if e == "bgr8":
            return self.bridge.imgmsg_to_cv2(msg, "bgr8")

        if e == "nv21":
            raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height * 3 // 2, msg.width)
            return cv2.cvtColor(raw, cv2.COLOR_YUV2BGR_NV21)

        return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)



    def scan_arr(self):
        if self.scan_msg is None:
            return None, None

        r = np.array(self.scan_msg.ranges, dtype=np.float32)
        a = self.scan_msg.angle_min + np.arange(len(r)) * self.scan_msg.angle_increment
        a = np.arctan2(np.sin(a), np.cos(a))

        bad = (
            (~np.isfinite(r))
            | (r <= self.scan_msg.range_min)
            | (r >= self.scan_msg.range_max)
        )
        r[bad] = np.inf
        return r, a

    def front_min(self, p_ang=None, p_dist=None):
        r, a = self.scan_arr()
        if r is None:
            return 5.0

        mask = np.abs(a) < math.radians(self.front_fov)


        if p_ang is not None and p_dist is not None and p_dist < 1.8:
            exc_bearing = np.abs(self.norm(a - float(p_ang))) < math.radians(22)
            exc_range = r > (p_dist - self.exc_margin)
            mask = mask & ~(exc_bearing & exc_range)

        fr = r[mask]
        fr = fr[np.isfinite(fr)]
        raw = float(np.min(fr)) if len(fr) > 0 else 5.0

        self._front_hist.append(raw)
        return float(np.min(list(self._front_hist)))

    def lidar_dist(self, ang, w_deg=18.0):
        r, a = self.scan_arr()
        if r is None:
            return None

        diff = np.abs(np.arctan2(np.sin(a - float(ang)), np.cos(a - float(ang))))
        vals = r[diff < math.radians(w_deg)]
        vals = vals[np.isfinite(vals)]

        if len(vals) == 0:
            return None

        d = float(np.median(vals))
        if 0.15 < d < 3.5:
            self._dist_hist.append(d)
            return float(np.median(list(self._dist_hist)))

        return None

    def estimate_dist(self, p_ang, bbox_y1, bbox_y2, frame_h):
        lidar_d = self.lidar_dist(p_ang)

        ratio = (bbox_y2 - bbox_y1) / max(1, frame_h)
        bbox_d = self.clamp(0.55 / ratio, 0.35, 3.0) if ratio > 0.08 else None

        if lidar_d is not None and bbox_d is not None:
            fused = 0.65 * lidar_d + 0.35 * bbox_d
        elif lidar_d is not None:
            fused = lidar_d
        elif bbox_d is not None:
            fused = bbox_d
        elif self.last_p_dist is not None:
            fused = self.last_p_dist
        else:
            fused = 1.4

        self.last_p_dist = fused
        return fused

    def side_clear(self):
        r, a = self.scan_arr()
        if r is None:
            return 1.0, 1.0

        left = r[(a > math.radians(10)) & (a < math.radians(105))]
        right = r[(a < -math.radians(10)) & (a > -math.radians(105))]

        left = left[np.isfinite(left)]
        right = right[np.isfinite(right)]

        lm = float(np.median(left)) if len(left) > 0 else 0.0
        rm = float(np.median(right)) if len(right) > 0 else 0.0
        return lm, rm

    def choose_avoid_dir(self):
        left, right = self.side_clear()

        if left > right + 0.10:
            self.avoid_dir = 1.0
        elif right > left + 0.10:
            self.avoid_dir = -1.0
        # if similar, keep previous direction


    def detect_persons(self, frame):
        results = self.model(frame, verbose=False, imgsz=320)
        persons = []

        for res in results:
            for box in res.boxes:
                if self.model.names[int(box.cls[0])] != "person":
                    continue

                conf = float(box.conf[0])
                if conf < self.conf_thresh:
                    continue

                x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
                persons.append((x1, y1, x2, y2, conf))

        return persons

    def person_hist(self, frame, box):
        x1, y1, x2, y2 = [int(v) for v in box[:4]]
        h, w = frame.shape[:2]

        x1 = self.clamp(x1, 0, w - 1)
        x2 = self.clamp(x2, 0, w - 1)
        y1 = self.clamp(y1, 0, h - 1)
        y2 = self.clamp(y2, 0, h - 1)

        if x2 <= x1 or y2 <= y1:
            return None


        bw = x2 - x1
        bh = y2 - y1
        tx1 = int(x1 + 0.20 * bw)
        tx2 = int(x2 - 0.20 * bw)
        ty1 = int(y1 + 0.25 * bh)
        ty2 = int(y1 + 0.75 * bh)

        crop = frame[ty1:ty2, tx1:tx2]
        if crop.size == 0:
            return None

        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        hist = cv2.calcHist([hsv], [0, 1], None, [24, 24], [0, 180, 0, 256])
        cv2.normalize(hist, hist, 0, 1, cv2.NORM_MINMAX)
        return hist

    def hist_score(self, frame, box):
        if self.target_hist is None:
            return 0.0

        h = self.person_hist(frame, box)
        if h is None:
            return 0.0

        s = cv2.compareHist(self.target_hist, h, cv2.HISTCMP_CORREL)
        return float(self.clamp((s + 1.0) / 2.0, 0.0, 1.0))

    def select_target(self, frame, persons):
        if not persons:
            return None

        h, w = frame.shape[:2]


        if self.target_hist is None:
            return max(persons, key=lambda b: (b[2] - b[0]) * (b[3] - b[1]))

        best = None
        best_score = -1.0

        for b in persons:
            x1, y1, x2, y2, conf = b
            area = max(1.0, (x2 - x1) * (y2 - y1))
            area_score = self.clamp(area / float(w * h * 0.35), 0.0, 1.0)

            cx = (x1 + x2) / 2.0
            ex = cx - w / 2.0
            ang = self.norm(-(ex / (w / 2.0)) * math.radians(self.camera_fov_deg) / 2.0)

            # Prefer target close to last known bearing, but do not overtrust this.
            bearing_diff = abs(self.norm(ang - self.last_p_ang))
            bearing_score = self.clamp(1.0 - bearing_diff / math.radians(45), 0.0, 1.0)

            color_score = self.hist_score(frame, b)

            score = 0.62 * color_score + 0.23 * bearing_score + 0.15 * area_score
            if score > best_score:
                best_score = score
                best = b

        return best

    def publish_detection(self, best, w, h):
        if best is None:
            return

        x1, y1, x2, y2, conf = best
        self.det_pub.publish(String(data=json.dumps({
            "detections": [{
                "bbox": [x1 / w, y1 / h, (x2 - x1) / w, (y2 - y1) / h],
                "label": "person",
                "conf": conf,
            }]
        })))


    def follow_ctrl(self, ang, dist):
        deg = math.degrees(ang)

        if dist is None:
            tv = 0.0
        elif dist < self.person_backoff_dist:
            tv = -0.025
        elif dist < self.too_close:
            tv = 0.0
        elif dist > self.follow_dead_high:
            tv = self.clamp(self.k_v * (dist - self.target_dist), 0.0, self.max_v)
        else:
            tv = 0.0

        if abs(deg) > self.rotate_only:
            # Rotate first if target is too far from center.
            tv = min(tv, 0.02)
            tw = self.clamp(self.k_w * ang, -self.max_w, self.max_w)
        elif abs(deg) < self.angle_dead:
            tw = 0.0
        else:
            ratio = abs(deg) / self.rotate_only
            tv = tv * (1.0 - ratio * 0.50)
            tw = self.clamp(self.k_w * ang, -self.max_w, self.max_w)


        if dist is not None and dist < self.target_dist + 0.3:
            tw *= 0.5

        return tv, tw

    def avoid_ctrl(self, p_ang, front):
        if self.state != ST_AVOID:
            self._set(ST_AVOID)
            self._avoid_dir_locked = False
            self._avoid_dwell = 0
            self.avoid_push_steps = 0

        # Do not flip direction every frame. Recheck occasionally only
        # when there is enough room. This reduces wall/column oscillation.
        if (not self._avoid_dir_locked) or (
            self._avoid_dwell > 0
            and self._avoid_dwell % 14 == 0
            and front > 0.45
        ):
            self.choose_avoid_dir()
            self._avoid_dir_locked = True

        self._avoid_dwell += 1
        avoid_dir = self.avoid_dir

        if front < self.critical:
            tv = -0.04
            tw = self.avoid_turn_w * avoid_dir
            self.avoid_push_steps = 0

        elif front < 0.45:
            tv = 0.030
            tw = self.avoid_turn_w * 0.60 * avoid_dir
            self.avoid_push_steps = 0

        elif front < self.obs_enter:
            tv = 0.040
            tw = self.avoid_turn_w * 0.70 * avoid_dir
            self.avoid_push_steps = 0

        elif front < self.obs_clear:
            tv = self.avoid_arc_v
            tw = self.avoid_arc_w * 0.40 * avoid_dir
            self.avoid_push_steps = 0

        else:
            if self.avoid_push_steps < self.avoid_push_steps_max:
                self.avoid_push_steps += 1
                tv = self.avoid_push_v
                tw = self.avoid_push_w
            else:
                tv = 0.0
                tw = 0.0

        self.get_logger().warn(
            f"[AVOID] front={front:.2f} dir={'L' if avoid_dir > 0 else 'R'} "
            f"dwell={self._avoid_dwell} push={self.avoid_push_steps}/{self.avoid_push_steps_max} "
            f"v={tv:.3f} w={tw:.3f}",
            throttle_duration_sec=0.5,
        )

        return tv, tw

    def reacq_ctrl(self, lost):
        if self.state != ST_REACQ:
            self._set(ST_REACQ)
            if abs(self.last_p_ang) > math.radians(5):
                self._search_dir = 1.0 if self.last_p_ang > 0 else -1.0
            else:
                self._search_dir = self.last_turn_dir

        if lost < self.grace_sec:
            return 0.0, 0.0

        if lost < self.search_sec:
            return 0.0, self.search_w_fast * self._search_dir

        return 0.0, self.search_w_slow * self._search_dir

    def _safety_override(self, twist, front):
        # Never move forward when the frontal obstacle is inside the hard safety zone.
        if front < self.critical and twist.linear.x > 0.0:
            twist.linear.x = 0.0
            self.prev_v = 0.0
        return twist

    def image_cb(self, msg):
        try:
            frame = self.to_bgr(msg)
        except Exception as e:
            self.get_logger().error(f"카메라 변환 실패: {e}", throttle_duration_sec=2.0)
            return

        h, w = frame.shape[:2]
        self.last_frame = frame.copy()

        persons = self.detect_persons(frame)
        best = self.select_target(frame, persons)
        self.publish_detection(best, w, h)

        if best is not None:
            self.last_best = best

        if not self.follow_on or not self.registered:
            self._set(ST_IDLE)
            self.publish_stop()
            if best is not None:
                self.get_logger().info(
                    "[IDLE] 사람 감지됨 — 'register' → 'follow' 명령 대기중",
                    throttle_duration_sec=3.0,
                )
            return

        now = time.time()
        vis = best is not None

        p_ang = self.last_p_ang
        p_dist = self.last_p_dist if self.last_p_dist else 1.4

        self._last_seen_before = self.last_seen

        if vis:
            x1, y1, x2, y2, _ = best
            cx = float((x1 + x2) / 2.0)
            ex = float(cx - w / 2.0)

            p_ang = self.norm(-(ex / (w / 2.0)) * math.radians(self.camera_fov_deg) / 2.0)
            p_dist = self.estimate_dist(p_ang, y1, y2, h)

            self.last_seen = now
            self.last_p_ang = p_ang

            if p_ang > 0.05:
                self.last_turn_dir = 1.0
            elif p_ang < -0.05:
                self.last_turn_dir = -1.0


        if self.state == ST_REACQ:
            if vis:
                self._reacq_confirm += 1
                if self._reacq_confirm < self.reacq_confirm_need:
                    vis = False
                    self.last_seen = self._last_seen_before
                else:
                    self._reacq_confirm = 0
            else:
                self._reacq_confirm = 0

        front = self.front_min(p_ang, p_dist)
        lost = now - self.last_seen

        if vis:
            in_avoid = self.state == ST_AVOID
            trigger = front < self.obs_enter
            hold = in_avoid and (
                front < self.obs_clear
                or self._avoid_dwell < self.avoid_dwell_min
                or self.avoid_push_steps < self.avoid_push_steps_max
            )

            if trigger or hold:
                tv, tw = self.avoid_ctrl(p_ang, front)
            else:
                if self.state != ST_FOLLOW:
                    self._set(ST_FOLLOW)
                    self._avoid_dir_locked = False
                    self._avoid_dwell = 0
                    self.avoid_push_steps = 0

                tv, tw = self.follow_ctrl(p_ang, p_dist)

            self.get_logger().info(
                f"[{self.state}] ang={math.degrees(p_ang):+.1f}° "
                f"dist={p_dist:.2f}m front={front:.2f}m "
                f"v={tv:.3f} w={tw:.3f}",
                throttle_duration_sec=0.5,
            )

        else:
            in_avoid = self.state == ST_AVOID

            # Finish avoidance and push before searching.
            # This is the key fix for "almost avoiding but stopping too early".
            if in_avoid and (
                front < self.obs_clear
                or self._avoid_dwell < self.avoid_dwell_min
                or self.avoid_push_steps < self.avoid_push_steps_max
            ):
                tv, tw = self.avoid_ctrl(self.last_p_ang, front)
            elif front < self.obs_enter:
                tv, tw = self.avoid_ctrl(self.last_p_ang, front)
            else:
                tv, tw = self.reacq_ctrl(lost)

            self.get_logger().info(
                f"[{self.state}] lost={lost:.1f}s front={front:.2f}m "
                f"confirm={self._reacq_confirm}/{self.reacq_confirm_need}",
                throttle_duration_sec=0.5,
            )

            if lost > self.search_timeout:
                self.follow_on = False
                self._set(ST_IDLE)
                self.publish_stop()
                self.get_logger().warn(f"[TIMEOUT] {lost:.0f}s 초과 — 추적 포기")
                return

        twist = self.smooth(tv, tw)
        twist = self._safety_override(twist, front)
        self.cmd_pub.publish(twist)


    def publish_stop(self):
        self.prev_v = 0.0
        self.prev_w = 0.0
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = HumanFollowNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("종료")
    finally:
        try:
            node.publish_stop()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

