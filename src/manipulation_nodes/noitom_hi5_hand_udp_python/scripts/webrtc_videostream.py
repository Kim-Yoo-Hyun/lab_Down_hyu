#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import socket
import sys
import os
import json
import time
import signal
import sys
from pprint import pprint
import tf
import rospy
import asyncio

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))

from UdpSenderForInfoToQuest3 import UdpSenderForInfoToQuest3
from webrtc_singaling_server import WebRTCSinglingServer 
from webrtcvideostreamclient import WebRTCVideoStreamClient

import netifaces as ni
import re
import numpy as np
import logging

from aiortc.codecs import vpx
vpx.DEFAULT_BITRATE = 8_000_000
vpx.MIN_BITRATE = 2_000_000
vpx.MAX_BITRATE = 16_000_000


# 루트 로거를 DEBUG 로 두면 aiortc/aioice/websockets 가 RTP 패킷 단위로 로그를 찍어
# 720p 20fps 에서 초당 수천 줄이 stdout 으로 나간다.
logging.basicConfig(level=logging.WARNING)
for _noisy in ("aiortc", "aioice", "websockets"):
    logging.getLogger(_noisy).setLevel(logging.WARNING)


_TARGET_BITRATE = int(os.environ.get("KUAVO_WEBRTC_BITRATE", 8_000_000))
for _mod_name in ("vpx", "h264"):
    try:
        _mod = __import__("aiortc.codecs." + _mod_name, fromlist=[_mod_name])
        _mod.DEFAULT_BITRATE = _TARGET_BITRATE
        _mod.MIN_BITRATE = _TARGET_BITRATE // 4
        _mod.MAX_BITRATE = _TARGET_BITRATE * 2
        print("[webrtc] %s target bitrate -> %.1f Mbps" % (_mod_name, _TARGET_BITRATE / 1e6))
    except Exception as _e:
        print("[webrtc] could not raise %s bitrate: %s" % (_mod_name, _e))


wait_webrtc_client_connect_timeout = 180  # seconds; default, overridden by ~webrtc_wait_timeout

class WebRTCServerAndVideoStreamClient:
    def __init__(self, camera_topic_for_video_stream, quest3_ip=None):
        self.camera_topic_for_video_stream = camera_topic_for_video_stream
        self.quest3_ip = quest3_ip
        self.web_rtc_signaling_server = None
        self.webrtc_video_stream_client = None
        self.udp_sender_send_webrtc_signaling_info = None

    def start(self):
        rospy.init_node('Leju_webrtc_VideoStream', anonymous=True)
        # Both waits below are configurable so a session can be launched before the
        # headset is awake.  Set either to 0 (or negative) to wait forever.
        self.wait_connect_timeout = float(
            rospy.get_param('~webrtc_wait_timeout', wait_webrtc_client_connect_timeout))
        self.video_init_timeout = float(rospy.get_param('~video_init_timeout', 10.0))
        rate = rospy.Rate(10)  # 10 Hz

        print("Starting to create WebRTC signaling server...")
        self.web_rtc_signaling_server = WebRTCSinglingServer()
        print("WebRTC signaling server created successfully.")

        self.web_rtc_signaling_server.start()
        print("Web_rtc_signaling_server_started: WebRTC signaling server started successfully.")

        self.webrtc_video_stream_client = WebRTCVideoStreamClient("127.0.0.1", self.camera_topic_for_video_stream)
        self.webrtc_video_stream_client.start()

        start_time = time.time()
        while self.webrtc_video_stream_client.width == 0 or self.webrtc_video_stream_client.height == 0:
            if self.video_init_timeout > 0 and time.time() - start_time > self.video_init_timeout:
                raise TimeoutError(
                    "Failed to initialize video stream within %g seconds" % self.video_init_timeout)
            time.sleep(0.1)

        width = self.webrtc_video_stream_client.width
        height = self.webrtc_video_stream_client.height
        print("\033[94m" + "Start ros node loop: Starting the rosnode loop" + "\033[0m")
        self.udp_sender_send_webrtc_signaling_info = self.BroadWebRtcAndCameraInfoToQuest3(width, height)

        start_time = time.time()
        while not rospy.is_shutdown():
            webrtc_clients_cnt = self.web_rtc_signaling_server.get_connected_clients_count()
            if webrtc_clients_cnt > 0:
                self.webrtc_video_stream_client.start_connect_webrtc_singal = True
                self.udp_sender_send_webrtc_signaling_info.stop()
                self.udp_sender_send_webrtc_signaling_info = None
                break
            elapsed_time = time.time() - start_time
            if self.wait_connect_timeout <= 0:
                # No countdown: report every 10 s instead of once per second.
                if int(elapsed_time) % 10 == 0:
                    print("Waiting for Quest3 to connect to webrtc server... {}s elapsed (no time limit)".format(int(elapsed_time)))
                time.sleep(1)
                continue
            if elapsed_time > self.wait_connect_timeout:
                print("\033[91mcarlos_webrtc_client_connect_timeout: Wait Quest3 connect to webrtc server: Timed out after {} seconds, keep waiting...\033[0m".format(self.wait_connect_timeout))
                start_time = time.time()
                continue
            remaining_time = int(self.wait_connect_timeout - elapsed_time)
            print("Waiting for Quest3 to connect to webrtc server... {} seconds remaining".format(remaining_time))
            time.sleep(1)  # add a 1-second sleep

    def BroadWebRtcAndCameraInfoToQuest3(self, width, height):
        webrtc_signaling_url = ":8765"
        ports = [10030, 10031, 10032, 10033, 10034, 10035, 10036, 10037, 10038, 10039, 10040]
        print(f"Webrtc signaling prot: {webrtc_signaling_url}")
        target_ips = [self.quest3_ip] if self.quest3_ip else None
        sender = UdpSenderForInfoToQuest3(ports, webrtc_signaling_url, width, height, target_ips=target_ips)
        sender.start()
        return sender

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 webrtc_videostream.py [<camera_topic_for_video_stream>] [quest3_ip]")
        sys.exit(1)

    camera_topic_for_video_stream = sys.argv[1]
    quest3_ip = sys.argv[2] if len(sys.argv) > 2 and "." in sys.argv[2] else None
    webrtc_server_and_video_stream_client = WebRTCServerAndVideoStreamClient(camera_topic_for_video_stream, quest3_ip)
    webrtc_server_and_video_stream_client.start()

    try:
        while not rospy.is_shutdown():
            rospy.sleep(1)

    except rospy.ROSInterruptException:
        rospy.loginfo("ROSInterruptException caught. Shutting down my_basic_node.")

    finally:
        rospy.loginfo("Leju_webrtc_VideoStream is shutting down.")
        if webrtc_server_and_video_stream_client.web_rtc_signaling_server is not None:
            webrtc_server_and_video_stream_client.web_rtc_signaling_server.stop()
        if webrtc_server_and_video_stream_client.udp_sender_send_webrtc_signaling_info is not None:
            webrtc_server_and_video_stream_client.udp_sender_send_webrtc_signaling_info.stop()

if __name__ == '__main__':
    main()
