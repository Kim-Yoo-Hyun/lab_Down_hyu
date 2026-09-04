#!/usr/bin/env python3
"""Combine the head and two wrist camera feeds for the Quest video stream."""

import threading

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, CompressedImage, Image


class QuestCameraMosaic:
    def __init__(self):
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.frames = {"head": None, "left": None, "right": None}
        self.frame_times = {"head": rospy.Time(0), "left": rospy.Time(0), "right": rospy.Time(0)}

        self.output_width = int(rospy.get_param("~output_width", 1280))
        self.output_height = int(rospy.get_param("~output_height", 720))
        self.output_fps = float(rospy.get_param("~output_fps", 20.0))
        self.stale_timeout = float(rospy.get_param("~stale_timeout", 1.0))
        # 카메라 드라이버가 다른 호스트에 있으면 raw 는 링크를 못 버틴다(720p bgr8 = 2.76MB/frame).
        # compressed 로 받으면 같은 링크에서 30Hz 가 그대로 도착한다.
        self.use_compressed = bool(rospy.get_param("~use_compressed", True))
        top_height = int(round(self.output_height * 2.0 / 3.0))
        default_positions = {
            "head": (self.output_width // 2, top_height // 2),
            "left": (self.output_width // 4, (top_height + self.output_height) // 2),
            "right": (self.output_width * 3 // 4, (top_height + self.output_height) // 2),
        }
        self.panel_layout = {
            name: {
                "x": int(rospy.get_param("~%s_x" % name, default_positions[name][0])),
                "y": int(rospy.get_param("~%s_y" % name, default_positions[name][1])),
                "scale": float(rospy.get_param("~%s_scale" % name, 1.0)),
            }
            for name in ("head", "left", "right")
        }
        self.debug_mode = bool(rospy.get_param("~debug_mode", False))
        self.debug_grid_step = int(rospy.get_param("~debug_grid_step", 100))
        self.show_labels = bool(rospy.get_param("~show_labels", False))
        self.output_prefix = rospy.get_param("~output_prefix", "/quest/mosaic").rstrip("/")

        if self.output_width < 2 or self.output_height < 2:
            raise ValueError("output_width and output_height must both be at least 2")
        if self.output_fps <= 0:
            raise ValueError("output_fps must be greater than zero")
        if self.debug_grid_step <= 0:
            raise ValueError("debug_grid_step must be greater than zero")
        for name, layout in self.panel_layout.items():
            if not 0.1 <= layout["scale"] <= 2.0:
                raise ValueError("%s_scale must be between 0.1 and 2.0" % name)

        topics = {
            "head": rospy.get_param("~head_topic", "/camera/color/image_raw"),
            "left": rospy.get_param("~left_topic", "/left_wrist_camera/color/image_raw"),
            "right": rospy.get_param("~right_topic", "/right_wrist_camera/color/image_raw"),
        }

        self.image_pub = rospy.Publisher(self.output_prefix + "/image_raw", Image, queue_size=1)
        self.info_pub = rospy.Publisher(self.output_prefix + "/camera_info", CameraInfo, queue_size=1)
        if self.use_compressed:
            self.subscribers = [
                rospy.Subscriber(topic.rstrip("/") + "/compressed", CompressedImage,
                                 self._compressed_callback, callback_args=name,
                                 queue_size=1, buff_size=16 * 1024 * 1024)
                for name, topic in topics.items()
            ]
        else:
            self.subscribers = [
                rospy.Subscriber(topic, Image, self._image_callback, callback_args=name,
                                 queue_size=1, buff_size=16 * 1024 * 1024)
                for name, topic in topics.items()
            ]

        rospy.loginfo(
            "Quest camera mosaic: head=%s left=%s right=%s output=%s (%dx%d @ %.1f Hz)",
            topics["head"], topics["left"], topics["right"], self.output_prefix,
            self.output_width, self.output_height, self.output_fps,
        )
        for name in ("head", "left", "right"):
            layout = self.panel_layout[name]
            rospy.loginfo(
                "Quest mosaic %s: scale=%.2f, x=%d px, y=%d px",
                name, layout["scale"], layout["x"], layout["y"],
            )
        rospy.loginfo("Quest mosaic debug mode: %s", self.debug_mode)
        rospy.loginfo("Quest mosaic camera labels: %s", self.show_labels)
        rospy.loginfo("Quest mosaic transport: %s, stale_timeout=%.2fs",
                      "compressed" if self.use_compressed else "raw", self.stale_timeout)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.output_fps), self._publish)

    def _compressed_callback(self, message, name):
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(message, desired_encoding="bgr8")
        except CvBridgeError as error:
            rospy.logerr_throttle(5.0, "Failed to decode %s compressed frame: %s", name, error)
            return

        with self.lock:
            self.frames[name] = frame
            self.frame_times[name] = rospy.Time.now()

    def _image_callback(self, message, name):
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except CvBridgeError as error:
            rospy.logerr_throttle(5.0, "Failed to decode %s camera frame: %s", name, error)
            return

        with self.lock:
            self.frames[name] = frame
            self.frame_times[name] = rospy.Time.now()

    @staticmethod
    def _fit(frame, width, height):
        canvas = np.zeros((height, width, 3), dtype=np.uint8)
        if frame is None or frame.size == 0:
            return canvas

        scale = min(float(width) / frame.shape[1], float(height) / frame.shape[0])
        resized_width = max(1, int(round(frame.shape[1] * scale)))
        resized_height = max(1, int(round(frame.shape[0] * scale)))
        resized = cv2.resize(frame, (resized_width, resized_height), interpolation=cv2.INTER_AREA)
        x = (width - resized_width) // 2
        y = (height - resized_height) // 2
        canvas[y:y + resized_height, x:x + resized_width] = resized
        return canvas

    @staticmethod
    def _label(frame, text, available):
        color = (255, 255, 255) if available else (80, 80, 255)
        suffix = "" if available else " - NO SIGNAL"
        cv2.rectangle(frame, (0, 0), (min(frame.shape[1], 360), 42), (0, 0, 0), -1)
        cv2.putText(frame, text + suffix, (12, 29), cv2.FONT_HERSHEY_SIMPLEX,
                    0.75, color, 2, cv2.LINE_AA)

    @staticmethod
    def _place(canvas, frame, x, y, width, height):
        if frame.shape[1] == width and frame.shape[0] == height:
            resized = frame
        else:
            resized = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)
        x = int(round(x))
        y = int(round(y))
        dst_x0 = max(0, x)
        dst_y0 = max(0, y)
        dst_x1 = min(canvas.shape[1], x + width)
        dst_y1 = min(canvas.shape[0], y + height)
        if dst_x0 >= dst_x1 or dst_y0 >= dst_y1:
            return
        src_x0 = dst_x0 - x
        src_y0 = dst_y0 - y
        src_x1 = src_x0 + (dst_x1 - dst_x0)
        src_y1 = src_y0 + (dst_y1 - dst_y0)
        canvas[dst_y0:dst_y1, dst_x0:dst_x1] = resized[src_y0:src_y1, src_x0:src_x1]

    def _compose_panels(self, head, left, right, top_height, bottom_height,
                        left_width, right_width):
        """Place each scaled panel around its absolute center coordinate."""
        canvas = np.zeros((self.output_height, self.output_width, 3), dtype=np.uint8)

        panels = (
            ("head", head, 0, 0, self.output_width, top_height),
            ("left", left, 0, top_height, left_width, bottom_height),
            ("right", right, left_width, top_height, right_width, bottom_height),
        )
        for name, frame, _base_x, _base_y, base_width, base_height in panels:
            layout = self.panel_layout[name]
            width = max(1, int(round(base_width * layout["scale"])))
            height = max(1, int(round(base_height * layout["scale"])))
            x = layout["x"] - width / 2.0
            y = layout["y"] - height / 2.0
            self._place(canvas, frame, x, y, width, height)
        return canvas

    def _draw_debug_coordinates(self, canvas):
        """Overlay an absolute pixel-coordinate grid and panel center markers."""
        red = (0, 0, 255)
        step = self.debug_grid_step
        for y in range(0, self.output_height, step):
            for x in range(0, self.output_width, step):
                cv2.circle(canvas, (x, y), 3, red, -1, cv2.LINE_AA)
                cv2.putText(
                    canvas, "(%d,%d)" % (x, y), (x + 5, y + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.32, red, 1, cv2.LINE_AA,
                )

        for name in ("head", "left", "right"):
            layout = self.panel_layout[name]
            center = (layout["x"], layout["y"])
            if 0 <= center[0] < self.output_width and 0 <= center[1] < self.output_height:
                cv2.drawMarker(canvas, center, red, cv2.MARKER_CROSS, 24, 2, cv2.LINE_AA)
                cv2.putText(
                    canvas, "%s (%d,%d)" % (name.upper(), center[0], center[1]),
                    (center[0] + 10, max(16, center[1] - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, red, 1, cv2.LINE_AA,
                )

    def _current_frame(self, name, now):
        frame = self.frames[name]
        fresh = frame is not None and (now - self.frame_times[name]).to_sec() <= self.stale_timeout
        return (frame if fresh else None), fresh

    def _publish(self, _event):
        now = rospy.Time.now()
        with self.lock:
            head, head_ok = self._current_frame("head", now)
            left, left_ok = self._current_frame("left", now)
            right, right_ok = self._current_frame("right", now)

        top_height = int(round(self.output_height * 2.0 / 3.0))
        bottom_height = self.output_height - top_height
        left_width = self.output_width // 2
        right_width = self.output_width - left_width

        head_panel = self._fit(head, self.output_width, top_height)
        left_panel = self._fit(left, left_width, bottom_height)
        right_panel = self._fit(right, right_width, bottom_height)
        if self.show_labels:
            self._label(head_panel, "HEAD", head_ok)
            self._label(left_panel, "LEFT WRIST", left_ok)
            self._label(right_panel, "RIGHT WRIST", right_ok)

        mosaic = self._compose_panels(
            head_panel, left_panel, right_panel,
            top_height, bottom_height, left_width, right_width,
        )
        if self.debug_mode:
            self._draw_debug_coordinates(mosaic)
        image_message = self.bridge.cv2_to_imgmsg(mosaic, encoding="bgr8")
        image_message.header.stamp = now
        image_message.header.frame_id = "quest_camera_mosaic"

        info_message = CameraInfo()
        info_message.header = image_message.header
        info_message.width = self.output_width
        info_message.height = self.output_height
        info_message.distortion_model = "plumb_bob"

        self.image_pub.publish(image_message)
        self.info_pub.publish(info_message)


def main():
    rospy.init_node("quest_camera_mosaic")
    QuestCameraMosaic()
    rospy.spin()


if __name__ == "__main__":
    main()
