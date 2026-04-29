#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
室外无人驾驶自行车视觉端 OpenCV 程序

功能：
1. 逆透视 + Canny + 滑动窗检测跑道线；
2. 连续发送循迹误差给 STM32；
3. 检测蓝色锥桶障碍物，重复发送事件直到 STM32 ACK；
4. 检测斑马线/人行道，重复发送事件直到 STM32 ACK；
5. 检测终点线，重复发送事件直到 STM32 ACK；
6. 接收 STM32 的 AVOID_EXIT，退出避障视觉状态，并向 STM32 ACK。

依赖：
    pip install opencv-python numpy pyserial

运行示例：
    python camera_bike_opencv.py --port COM7 --camera 0 --show
    python camera_bike_opencv.py --port /dev/ttyUSB0 --camera 0 --show

"""

import argparse
import struct
import time
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple

import cv2
import numpy as np

try:
    import serial
except ImportError:
    serial = None


# =========================
# 通信协议定义
# =========================
HEAD1 = 0xA5
HEAD2 = 0x5A

MSG_LINE = 0x01
MSG_OBSTACLE = 0x02
MSG_CROSSWALK = 0x03
MSG_FINISH = 0x04
MSG_HEARTBEAT = 0x05

MSG_ACK = 0x80
MSG_MCU_STATE = 0x81
MSG_AVOID_EXIT = 0x82
MSG_MCU_HEARTBEAT = 0x83

SIDE_UNKNOWN = 0
SIDE_LEFT = 1     # 障碍物在左侧
SIDE_RIGHT = 2    # 障碍物在右侧

VISION_CENTER = 0
VISION_FOLLOW_LEFT = 1
VISION_FOLLOW_RIGHT = 2


def xor_checksum(frame_body: bytes) -> int:
    """校验范围：TYPE, SEQ, LEN, PAYLOAD。"""
    v = 0
    for b in frame_body:
        v ^= b
    return v & 0xFF


class SerialProtocol:
    """A5 5A TYPE SEQ LEN PAYLOAD XOR"""

    def __init__(self, port: Optional[str], baudrate: int = 115200):
        self.ser = None
        if port and port.lower() != "none":
            if serial is None:
                raise RuntimeError("未安装 pyserial，请先 pip install pyserial")
            self.ser = serial.Serial(port, baudrate=baudrate, timeout=0)

        self.rx_state = 0
        self.rx_type = 0
        self.rx_seq = 0
        self.rx_len = 0
        self.rx_payload = bytearray()

    def send_frame(self, msg_type: int, seq: int, payload: bytes = b""):
        if len(payload) > 64:
            raise ValueError("payload too long")
        body = bytes([msg_type & 0xFF, seq & 0xFF, len(payload) & 0xFF]) + payload
        chk = xor_checksum(body)
        frame = bytes([HEAD1, HEAD2]) + body + bytes([chk])
        if self.ser:
            self.ser.write(frame)

    def recv_frames(self):
        """非阻塞解析，返回 [(type, seq, payload), ...]"""
        out = []
        if not self.ser:
            return out

        data = self.ser.read(128)
        for b in data:
            if self.rx_state == 0:
                if b == HEAD1:
                    self.rx_state = 1
            elif self.rx_state == 1:
                if b == HEAD2:
                    self.rx_state = 2
                elif b == HEAD1:
                    self.rx_state = 1
                else:
                    self.rx_state = 0
            elif self.rx_state == 2:
                self.rx_type = b
                self.rx_state = 3
            elif self.rx_state == 3:
                self.rx_seq = b
                self.rx_state = 4
            elif self.rx_state == 4:
                self.rx_len = b
                self.rx_payload = bytearray()
                if self.rx_len == 0:
                    self.rx_state = 6
                elif self.rx_len <= 64:
                    self.rx_state = 5
                else:
                    self.rx_state = 0
            elif self.rx_state == 5:
                self.rx_payload.append(b)
                if len(self.rx_payload) >= self.rx_len:
                    self.rx_state = 6
            elif self.rx_state == 6:
                body = bytes([self.rx_type, self.rx_seq, self.rx_len]) + bytes(self.rx_payload)
                if xor_checksum(body) == b:
                    out.append((self.rx_type, self.rx_seq, bytes(self.rx_payload)))
                self.rx_state = 0
        return out


class TrackState(Enum):
    CENTER = 0
    LEFT = 1
    RIGHT = 2


@dataclass
class VisionConfig:
    img_w: int = 640
    img_h: int = 480

    # 逆透视参数：实车必须微调
    src_points: Tuple[Tuple[int, int], Tuple[int, int], Tuple[int, int], Tuple[int, int]] = (
        (172, 330), (461, 330), (75, 475), (546, 475)
    )
    dst_x: int = 220
    dst_y: int = 250
    dst_w: int = 200
    dst_h: int = 230

    # 预处理
    median_ksize: int = 9
    canny_low: int = 40
    canny_high: int = 50
    edge_mask_left: int = 160
    edge_mask_right: int = 480

    # 霍夫/滑窗参数
    hough_threshold: int = 230
    hough_y: int = 420
    hough_angle_limit_deg: float = 35.0
    find_delta_x: int = 180
    nwindows: int = 8
    margin: int = 35
    minpix: int = 20

    # 跟踪偏移
    shifted_x: int = 52       # 跟左线/右线时向赛道内部偏移的像素，实车调
    avoid_shift_x: int = 88   # 避障时更靠另一侧的偏移

    # 蓝色锥桶 HSV，强烈建议现场用 trackbar 调
    blue_low: Tuple[int, int, int] = (90, 80, 40)
    blue_high: Tuple[int, int, int] = (130, 255, 255)
    blue_min_area: int = 180
    blue_min_w: int = 10
    blue_min_h: int = 12
    obstacle_stable_frames: int = 3

    # 斑马线检测：在逆透视图下检测多条横向白色条纹
    crosswalk_roi_y1: int = 250
    crosswalk_roi_y2: int = 470
    crosswalk_min_stripes: int = 3
    crosswalk_min_width: int = 120
    crosswalk_stable_frames: int = 5

    # 终点线检测：默认检测底部宽横线，可按实际终点标志修改
    finish_roi_y1: int = 330
    finish_roi_y2: int = 470
    finish_min_width: int = 220
    finish_stable_frames: int = 4

    # 串口发送频率
    line_send_period_s: float = 0.02     # 50Hz
    event_resend_period_s: float = 0.03  # 事件重发，直到 ACK


class BikeVision:
    def __init__(self, cfg: VisionConfig, proto: SerialProtocol, show: bool = False):
        self.cfg = cfg
        self.proto = proto
        self.show = show

        self.img_size = (cfg.img_w, cfg.img_h)
        self.M, self.M_inv = self._build_perspective()

        self.track_state = TrackState.CENTER
        self.dynamic_center_x = cfg.img_w // 2
        self.leftx_mean = cfg.img_w // 2 - 90
        self.rightx_mean = cfg.img_w // 2 + 90
        self.left_fit = None
        self.right_fit = None
        self.last_error = 0
        self.error_flag = True

        self.seq = 1
        self.pending_event = None  # dict(type, seq, payload, last_send)
        self.last_line_send_t = 0.0
        self.last_hb_t = 0.0

        self.obstacle_cnt = 0
        self.crosswalk_cnt = 0
        self.finish_cnt = 0

        self.last_obstacle_side = SIDE_UNKNOWN
        self.avoid_mode = False
        self.avoid_enter_t = 0.0
        self.crosswalk_latched = False
        self.finish_latched = False

        self.debug_img = None
        self.warp_img = None
        self.edges = None

    def _build_perspective(self):
        c = self.cfg
        src = np.float32(c.src_points)
        dst = np.float32([
            [c.dst_x, c.dst_y],
            [c.dst_x + c.dst_w, c.dst_y],
            [c.dst_x, c.dst_y + c.dst_h],
            [c.dst_x + c.dst_w, c.dst_y + c.dst_h],
        ])
        return cv2.getPerspectiveTransform(src, dst), cv2.getPerspectiveTransform(dst, src)

    def preprocess(self, img):
        c = self.cfg
        img = cv2.resize(img, self.img_size)
        img_blur = cv2.medianBlur(img, c.median_ksize)
        warp = cv2.warpPerspective(img_blur, self.M, self.img_size)
        edges = cv2.Canny(warp, c.canny_high, c.canny_low, apertureSize=3)
        edges = cv2.dilate(edges, np.ones((3, 3), np.uint8), iterations=2)
        mask = np.zeros((c.img_h, c.img_w), dtype=np.uint8)
        cv2.rectangle(mask, (c.edge_mask_left, 0), (c.edge_mask_right, c.img_h), 255, cv2.FILLED)
        edges = cv2.bitwise_and(edges, edges, mask=mask)
        self.warp_img = warp
        self.edges = edges
        return warp, edges

    def hough_start_points(self):
        c = self.cfg
        line_x = []
        lines = cv2.HoughLines(self.edges, 1, np.pi / 180, threshold=c.hough_threshold)
        if lines is not None:
            for line in lines:
                rho, theta = line[0]
                theta_deg = np.degrees(theta)
                if theta_deg > 90:
                    theta_deg = 180 - theta_deg
                if abs(theta_deg) > c.hough_angle_limit_deg:
                    continue
                if abs(np.sin(theta)) < 1e-5:
                    x = int(rho)
                else:
                    m = -1.0 / np.tan(theta)
                    b = rho / np.sin(theta)
                    if abs(m) < 1e-5:
                        continue
                    x = int((c.hough_y - b) / m)
                if 0 <= x < c.img_w:
                    line_x.append(x)

        left_candidates = [x for x in line_x if self.dynamic_center_x - c.find_delta_x < x < self.dynamic_center_x]
        right_candidates = [x for x in line_x if self.dynamic_center_x < x < self.dynamic_center_x + c.find_delta_x]

        if left_candidates:
            self.leftx_mean = int(np.mean(left_candidates))
        if right_candidates:
            self.rightx_mean = int(np.mean(right_candidates))

    def sliding_window_fit(self):
        c = self.cfg
        nonzero = self.edges.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        window_h = int(c.img_h / c.nwindows)

        left_inds = []
        right_inds = []
        lx = self.leftx_mean
        rx = self.rightx_mean

        for window in range(c.nwindows):
            y_low = c.img_h - (window + 1) * window_h
            y_high = c.img_h - window * window_h

            left_low = lx - c.margin
            left_high = lx + c.margin
            right_low = rx - c.margin
            right_high = rx + c.margin

            good_left = ((nonzeroy >= y_low) & (nonzeroy < y_high) &
                         (nonzerox >= left_low) & (nonzerox < left_high)).nonzero()[0]
            good_right = ((nonzeroy >= y_low) & (nonzeroy < y_high) &
                          (nonzerox >= right_low) & (nonzerox < right_high)).nonzero()[0]

            if len(good_left) > c.minpix:
                lx = int(np.mean(nonzerox[good_left]))
            if len(good_right) > c.minpix:
                rx = int(np.mean(nonzerox[good_right]))

            left_inds.append(good_left)
            right_inds.append(good_right)

            if self.show:
                cv2.rectangle(self.warp_img, (left_low, y_low), (left_high, y_high), (0, 255, 0), 2)
                cv2.rectangle(self.warp_img, (right_low, y_low), (right_high, y_high), (0, 255, 0), 2)

        self.leftx_mean = lx
        self.rightx_mean = rx

        try:
            left_inds = np.concatenate(left_inds)
            right_inds = np.concatenate(right_inds)
            leftx, lefty = nonzerox[left_inds], nonzeroy[left_inds]
            rightx, righty = nonzerox[right_inds], nonzeroy[right_inds]
            if len(leftx) > 30:
                self.left_fit = np.polyfit(lefty, leftx, 2)
            if len(rightx) > 30:
                self.right_fit = np.polyfit(righty, rightx, 2)
            self.error_flag = self.left_fit is None and self.right_fit is None
        except Exception:
            self.error_flag = True

    def update_dynamic_center(self):
        """简化版动态中点：避障时根据当前跟踪边线移动寻线中心。"""
        if self.track_state == TrackState.CENTER:
            self.dynamic_center_x = self.cfg.img_w // 2
            return
        lane_half = max(60, int(abs(self.rightx_mean - self.leftx_mean) / 2))
        if self.track_state == TrackState.LEFT:
            self.dynamic_center_x = int(self.leftx_mean + lane_half)
        elif self.track_state == TrackState.RIGHT:
            self.dynamic_center_x = int(self.rightx_mean - lane_half)
        self.dynamic_center_x = int(np.clip(self.dynamic_center_x, 120, 520))

    def compute_error(self) -> Optional[int]:
        c = self.cfg
        y = c.hough_y
        x_mean = None
        shift = c.avoid_shift_x if self.avoid_mode else c.shifted_x

        try:
            if self.track_state == TrackState.LEFT and self.left_fit is not None:
                fit = np.copy(self.left_fit)
                fit[2] += shift
                x_mean = fit[0] * y * y + fit[1] * y + fit[2]
            elif self.track_state == TrackState.RIGHT and self.right_fit is not None:
                fit = np.copy(self.right_fit)
                fit[2] -= shift
                x_mean = fit[0] * y * y + fit[1] * y + fit[2]
            elif self.left_fit is not None and self.right_fit is not None:
                mid_fit = (self.left_fit + self.right_fit) / 2.0
                x_mean = mid_fit[0] * y * y + mid_fit[1] * y + mid_fit[2]
            elif self.left_fit is not None:
                x_mean = self.left_fit[0] * y * y + self.left_fit[1] * y + self.left_fit[2] + shift
            elif self.right_fit is not None:
                x_mean = self.right_fit[0] * y * y + self.right_fit[1] * y + self.right_fit[2] - shift

            if x_mean is None:
                return None
            err = int(x_mean - c.img_w // 2)
            err = int(self.last_error * 0.3 + err * 0.7)
            self.last_error = err

            if self.show:
                cv2.circle(self.warp_img, (int(x_mean), y), 8, (0, 0, 255), -1)
                cv2.line(self.warp_img, (c.img_w // 2, 0), (c.img_w // 2, c.img_h), (255, 0, 0), 2)
            return err
        except Exception:
            return None

    def detect_blue_cone(self, img) -> Tuple[bool, int, int]:
        """返回 detected, side, confidence。"""
        if self.left_fit is None or self.right_fit is None:
            return False, SIDE_UNKNOWN, 0

        c = self.cfg
        # 使用原图下半部分中间区域检测蓝色锥桶，简单稳妥；实车可根据左右线反透视精确裁剪。
        y1, y2 = 180, 430
        x1, x2 = 120, 520
        roi = img[y1:y2, x1:x2]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(c.blue_low), np.array(c.blue_high))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations=1)
        mask = cv2.dilate(mask, np.ones((5, 5), np.uint8), iterations=2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        blocks = []
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            area = cv2.contourArea(cnt)
            if area >= c.blue_min_area and w >= c.blue_min_w and h >= c.blue_min_h:
                blocks.append((x, y, w, h, area))

        if not blocks:
            self.obstacle_cnt = 0
            return False, SIDE_UNKNOWN, 0

        # 取最大蓝色色块
        x, y, w, h, area = max(blocks, key=lambda t: t[4])
        cx = x1 + x + w // 2
        side = SIDE_LEFT if cx < c.img_w // 2 else SIDE_RIGHT
        confidence = int(np.clip(area / 800.0 * 100, 0, 100))

        if side == self.last_obstacle_side:
            self.obstacle_cnt += 1
        else:
            self.obstacle_cnt = 1
            self.last_obstacle_side = side

        if self.show:
            cv2.rectangle(img, (x1 + x, y1 + y), (x1 + x + w, y1 + y + h), (255, 0, 0), 2)
            cv2.putText(img, f"BLUE side={side} cnt={self.obstacle_cnt}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        return self.obstacle_cnt >= c.obstacle_stable_frames, side, confidence

    def detect_crosswalk(self) -> Tuple[bool, int]:
        c = self.cfg
        roi = self.warp_img[c.crosswalk_roi_y1:c.crosswalk_roi_y2, c.edge_mask_left:c.edge_mask_right]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, th = cv2.threshold(gray, 165, 255, cv2.THRESH_BINARY)
        th = cv2.morphologyEx(th, cv2.MORPH_OPEN, np.ones((3, 9), np.uint8), iterations=1)
        contours, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        stripe_cnt = 0
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            if w > c.crosswalk_min_width and 4 <= h <= 35:
                stripe_cnt += 1
        if stripe_cnt >= c.crosswalk_min_stripes:
            self.crosswalk_cnt += 1
        else:
            self.crosswalk_cnt = 0
        confidence = int(np.clip(stripe_cnt / max(1, c.crosswalk_min_stripes) * 100, 0, 100))
        return self.crosswalk_cnt >= c.crosswalk_stable_frames, confidence

    def detect_finish(self) -> Tuple[bool, int]:
        c = self.cfg
        roi = self.warp_img[c.finish_roi_y1:c.finish_roi_y2, c.edge_mask_left:c.edge_mask_right]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        # 默认检测黄色/白色宽横条，按实际终点线颜色调。
        yellow = cv2.inRange(hsv, np.array([15, 60, 60]), np.array([40, 255, 255]))
        white = cv2.inRange(hsv, np.array([0, 0, 170]), np.array([180, 70, 255]))
        mask = cv2.bitwise_or(yellow, white)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 15), np.uint8), iterations=2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        wide = 0
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            if w >= c.finish_min_width and h >= 8:
                wide += 1
        if wide > 0:
            self.finish_cnt += 1
        else:
            self.finish_cnt = 0
        confidence = 100 if wide > 0 else 0
        return self.finish_cnt >= c.finish_stable_frames, confidence

    def next_seq(self):
        self.seq = (self.seq + 1) & 0xFF
        if self.seq == 0:
            self.seq = 1
        return self.seq

    def set_pending_event(self, msg_type: int, payload: bytes):
        if self.pending_event is not None:
            return
        self.pending_event = {
            "type": msg_type,
            "seq": self.next_seq(),
            "payload": payload,
            "last_send": 0.0,
        }

    def handle_rx(self):
        for msg_type, seq, payload in self.proto.recv_frames():
            if msg_type == MSG_ACK and len(payload) >= 2:
                ack_type, ack_seq = payload[0], payload[1]
                if self.pending_event and ack_type == self.pending_event["type"] and ack_seq == self.pending_event["seq"]:
                    self.pending_event = None
            elif msg_type == MSG_AVOID_EXIT:
                # STM32 通知视觉端退出避障，摄像头 ACK 后恢复中线循迹。
                self.proto.send_frame(MSG_ACK, seq, bytes([MSG_AVOID_EXIT, seq]))
                self.avoid_mode = False
                self.track_state = TrackState.CENTER
                self.dynamic_center_x = self.cfg.img_w // 2
                self.obstacle_cnt = 0

    def send_line(self, err: int, confidence: int = 100):
        now = time.time()
        if now - self.last_line_send_t < self.cfg.line_send_period_s:
            return
        self.last_line_send_t = now
        vision_state = VISION_CENTER
        if self.track_state == TrackState.LEFT:
            vision_state = VISION_FOLLOW_LEFT
        elif self.track_state == TrackState.RIGHT:
            vision_state = VISION_FOLLOW_RIGHT
        payload = struct.pack(">hBB", int(err), int(confidence) & 0xFF, vision_state & 0xFF)
        self.proto.send_frame(MSG_LINE, 0, payload)

    def resend_pending_event(self):
        if not self.pending_event:
            return
        now = time.time()
        if now - self.pending_event["last_send"] >= self.cfg.event_resend_period_s:
            self.proto.send_frame(self.pending_event["type"], self.pending_event["seq"], self.pending_event["payload"])
            self.pending_event["last_send"] = now

    def send_heartbeat(self):
        now = time.time()
        if now - self.last_hb_t >= 0.2:
            self.last_hb_t = now
            self.proto.send_frame(MSG_HEARTBEAT, 0, b"\x01")

    def process_frame(self, img):
        self.debug_img = img.copy()
        self.preprocess(img)
        self.hough_start_points()
        self.sliding_window_fit()
        self.update_dynamic_center()
        err = self.compute_error()
        if err is not None:
            self.send_line(err, 100 if not self.error_flag else 20)

        # 终点优先级最高
        finish, finish_conf = self.detect_finish()
        if finish and not self.finish_latched:
            self.finish_latched = True
            self.set_pending_event(MSG_FINISH, struct.pack(">hBB", err or 0, SIDE_UNKNOWN, finish_conf))

        # 人行道次优先级；终点已经锁存后不再处理其他事件
        crosswalk, cross_conf = self.detect_crosswalk()
        if crosswalk and not self.crosswalk_latched and not self.finish_latched:
            self.crosswalk_latched = True
            self.set_pending_event(MSG_CROSSWALK, struct.pack(">hBB", err or 0, SIDE_UNKNOWN, cross_conf))

        # 蓝色锥桶障碍物；只在正常中线状态下触发一次
        obstacle, side, obs_conf = self.detect_blue_cone(self.debug_img)
        if obstacle and not self.avoid_mode and not self.finish_latched and not self.crosswalk_latched:
            self.avoid_mode = True
            self.avoid_enter_t = time.time()
            # 如果障碍物在左边，切到跟右线；在右边，切到跟左线。
            self.track_state = TrackState.RIGHT if side == SIDE_LEFT else TrackState.LEFT
            payload = struct.pack(">hBB", err or 0, side & 0xFF, obs_conf & 0xFF)
            self.set_pending_event(MSG_OBSTACLE, payload)

        self.resend_pending_event()
        self.send_heartbeat()

        if self.show:
            cv2.putText(self.debug_img, f"state={self.track_state.name} avoid={self.avoid_mode} err={err}",
                        (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            if self.pending_event:
                cv2.putText(self.debug_img, f"pending type=0x{self.pending_event['type']:02X} seq={self.pending_event['seq']}",
                            (20, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.imshow("camera", self.debug_img)
            cv2.imshow("warp", self.warp_img)
            cv2.imshow("edges", self.edges)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="None", help="串口，例如 COM7 或 /dev/ttyUSB0；None 表示只显示不发送")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--video", default=None, help="也可以输入视频文件路径调试")
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args()

    proto = SerialProtocol(args.port, args.baud)
    vision = BikeVision(VisionConfig(), proto, show=args.show)

    cap = cv2.VideoCapture(args.video if args.video else args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    if not cap.isOpened():
        raise RuntimeError("无法打开摄像头/视频")

    while True:
        vision.handle_rx()
        ok, frame = cap.read()
        if not ok:
            break
        vision.process_frame(frame)
        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord('q'):
            break
        elif key == ord('c'):
            # 调试时手动清除一次人行道锁存
            vision.crosswalk_latched = False
        elif key == ord('f'):
            vision.finish_latched = False

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
