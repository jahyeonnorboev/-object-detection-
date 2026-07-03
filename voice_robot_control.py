#!/usr/bin/env python3
import json
import subprocess
import time

from vosk import Model, KaldiRecognizer

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


KO_MODEL = "/home/ns2022427138/vosk_models/vosk-model-small-ko-0.22"
EN_MODEL = "/home/ns2022427138/vosk_models/vosk-model-small-en-us-0.15"
MIC_DEVICE = "plughw:1,0"

MODE_MANUAL = "manual"
MODE_FOLLOW = "follow"
MODE_NAVIGATION = "navigation"


class VoiceRobotControl(Node):
    def __init__(self):
        super().__init__("voice_robot_control")

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.yolo_pub = self.create_publisher(String, "/yolo_command", qos)
        self.nav_pub = self.create_publisher(String, "/voice_nav_command", 10)

        self.mode = MODE_MANUAL
        self.linear = 0.0
        self.angular = 0.0
        self.last_cmd_time = time.time()
        self.safety_timeout = 5.0

        self.timer = self.create_timer(0.1, self.publish_motion)

        self.get_logger().info("Voice Robot Control Ready")
        self.get_logger().info("Modes: manual / follow / navigation")

    def publish_motion(self):
        if self.mode != MODE_MANUAL:
            return

        if time.time() - self.last_cmd_time > self.safety_timeout:
            self.linear = 0.0
            self.angular = 0.0

        msg = Twist()
        msg.linear.x = self.linear
        msg.angular.z = self.angular
        self.cmd_pub.publish(msg)

    def set_motion(self, linear, angular):
        if self.mode != MODE_MANUAL:
            self.get_logger().warn(f"cmd_vel blocked in {self.mode} mode")
            return

        self.linear = linear
        self.angular = angular
        self.last_cmd_time = time.time()

    def stop_cmd_vel_once(self):
        msg = Twist()
        self.cmd_pub.publish(msg)
        self.linear = 0.0
        self.angular = 0.0
        self.last_cmd_time = time.time()

    def set_mode(self, mode):
        if self.mode != mode:
            self.get_logger().info(f"MODE: {self.mode} -> {mode}")
            self.mode = mode

    def yolo_cmd(self, cmd):
        self.yolo_pub.publish(String(data=cmd))
        self.get_logger().info(f"/yolo_command <- {cmd}")

    def nav_cmd(self, cmd):
        self.nav_pub.publish(String(data=cmd))
        self.get_logger().info(f"/voice_nav_command <- {cmd}")

    def stop_all(self):
        self.nav_cmd("stop")
        self.yolo_cmd("stop")
        self.stop_cmd_vel_once()
        self.set_mode(MODE_MANUAL)


def has_any(text, words):
    t = text.lower()
    return any(w.lower() in t for w in words)


def normalize_text(text):
    t = text.lower().strip()
    t = t.replace("royce", "")
    t = t.replace("로이스", "")
    t = t.replace("로이 스", "")
    return t.strip()


def handle_text(node, text, lang):
    if not text:
        return

    print(f"🗣 [{lang}] {text}")
    t = normalize_text(text)

    if has_any(t, ["stop", "pause", "멈춰", "정지", "스톱"]):
        node.stop_all()
        print(" STOP: nav + yolo + cmd_vel")
        return

    # NAVIGATION 명령
    if has_any(t, ["go home", "go to home", "home", "홈", "집으로"]):
        node.yolo_cmd("stop")
        node.stop_cmd_vel_once()
        node.set_mode(MODE_NAVIGATION)
        node.nav_cmd("go:home")
        print(" NAVIGATION: go home")
        return

    if has_any(t, ["go door", "go to door", "door", "문", "출입문"]):
        node.yolo_cmd("stop")
        node.stop_cmd_vel_once()
        node.set_mode(MODE_NAVIGATION)
        node.nav_cmd("go:door")
        print(" NAVIGATION: go door")
        return

    if has_any(t, ["go desk", "go to desk", "desk", "책상"]):
        node.yolo_cmd("stop")
        node.stop_cmd_vel_once()
        node.set_mode(MODE_NAVIGATION)
        node.nav_cmd("go:desk")
        print(" NAVIGATION: go desk")
        return

    if has_any(t, ["go chair", "go to chair", "chair", "의자"]):
        node.yolo_cmd("stop")
        node.stop_cmd_vel_once()
        node.set_mode(MODE_NAVIGATION)
        node.nav_cmd("go:chair")
        print(" NAVIGATION: go chair")
        return

    # 모드 전환
    if has_any(t, ["manual mode", "manual", "수동 모드", "수동"]):
        node.nav_cmd("stop")
        node.yolo_cmd("stop")
        node.stop_cmd_vel_once()
        node.set_mode(MODE_MANUAL)
        print(" manual mode")
        return

    if has_any(t, ["tracking mode", "follow mode", "추적 모드"]):
        node.nav_cmd("stop")
        node.stop_cmd_vel_once()
        node.set_mode(MODE_FOLLOW)
        print(" follow mode")
        return

    # FOLLOW 명령
    if has_any(t, ["follow me", "follow", "start following", "따라와", "추적 시작"]):
        node.nav_cmd("stop")
        node.stop_cmd_vel_once()
        node.set_mode(MODE_FOLLOW)

        node.yolo_cmd("register")
        time.sleep(0.7)
        node.yolo_cmd("follow")

        print(" FOLLOW: register + follow")
        return

    if has_any(t, ["register", "person register", "사람 등록", "등록"]):
        node.nav_cmd("stop")
        node.stop_cmd_vel_once()
        node.set_mode(MODE_FOLLOW)
        node.yolo_cmd("register")
        print(" 사람 등록")
        return

    if has_any(t, ["reset", "초기화", "리셋"]):
        node.yolo_cmd("reset")
        print(" YOLO reset")
        return

    # 수동 이동
    if node.mode == MODE_MANUAL:
        if has_any(t, ["forward", "go up", "go straight", "앞으로", "전진"]):
            node.set_motion(0.10, 0.0)
            print("⬆ manual forward")
            return

        if has_any(t, ["back", "go back", "come back", "backward", "뒤로", "후진"]):
            node.set_motion(-0.08, 0.0)
            print("⬇ manual back")
            return

        if has_any(t, ["turn left", "left", "왼쪽", "좌회전"]):
            node.set_motion(0.0, 0.45)
            print("↩ manual left")
            return

        if has_any(t, ["turn right", "right", "오른쪽", "우회전"]):
            node.set_motion(0.0, -0.45)
            print("↪ manual right")
            return

    else:
        if has_any(t, ["forward", "back", "left", "right", "앞으로", "뒤로", "왼쪽", "오른쪽"]):
            print(f" manual cmd ignored in {node.mode} mode")
            return

    print("명령 매칭 없음")


def main():
    rclpy.init()
    node = VoiceRobotControl()

    print("모델 로딩 중...")
    ko_model = Model(KO_MODEL)
    en_model = Model(EN_MODEL)

    ko_rec = KaldiRecognizer(ko_model, 16000)
    en_rec = KaldiRecognizer(en_model, 16000)

    print(" 음성 제어 시작")
    print("예시:")
    print("  Royce go home")
    print("  Royce go to door")
    print("  Royce go to desk")
    print("  Royce follow me")
    print("  Royce stop")
    print("  manual / forward / back / left / right")

    process = subprocess.Popen(
        ["arecord", "-D", MIC_DEVICE, "-f", "S16_LE", "-r", "16000", "-c", "1", "-q"],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
    )

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            data = process.stdout.read(4000)

            if len(data) == 0:
                continue

            if ko_rec.AcceptWaveform(data):
                text = json.loads(ko_rec.Result()).get("text", "").strip()
                if text:
                    handle_text(node, text, "KO")

            if en_rec.AcceptWaveform(data):
                text = json.loads(en_rec.Result()).get("text", "").strip()
                if text:
                    handle_text(node, text, "EN")

    except KeyboardInterrupt:
        print("\n종료")

    finally:
        process.terminate()
        node.stop_all()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
