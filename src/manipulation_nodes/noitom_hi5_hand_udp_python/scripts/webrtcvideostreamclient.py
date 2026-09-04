import cv2
import asyncio
import json
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack, RTCIceCandidate
from aiortc.contrib.signaling import BYE
import websockets
from aiortc.contrib.media import MediaStreamTrack
from aiortc.mediastreams import VIDEO_CLOCK_RATE, VIDEO_TIME_BASE
from datetime import datetime
from av import VideoFrame
import av
import fractions
import sys
import subprocess
import rospy
from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge
import signal
import threading
import time

class WebRTCVideoStreamClient:
    def __init__(self, server_ip, ros_topic):
        self.server_ip = server_ip
        self.ros_topic = f"{ros_topic}/image_raw"
        self.ros_camera_info_topic = f"{ros_topic}/camera_info"

        self.wait_for_topics(timeout=30.0)

        self.bridge = CvBridge()
        self.latest_frame = None
        self.frame_seq = 0
        self.pc = None
        self.width = 0
        self.height = 0
        self.start_connect_webrtc_singal = False

    def topic_exists(self, topic_name):
        """
        Directly check if a given ROS topic exists.

        Args:
            topic_name (str): The name of the topic to check (e.g., '/camera/image_raw').

        Returns:
            bool: True if the topic exists, False otherwise.
        """
        try:
            # Retrieve the list of all published topics
            published_topics = rospy.get_published_topics()

            # Extract only the topic names for efficient searching
            topic_names = [topic for topic, _ in published_topics]

            # Check if the target topic is in the list
            return topic_name in topic_names
        except rospy.ROSException as e:
            rospy.logerr("Failed to retrieve published topics: %s", e)
            return False

    def wait_for_topics(self, timeout):
        """Wait for camera publishers so nodes started by the same launch can initialize."""
        deadline = time.monotonic() + timeout
        missing = []
        while not rospy.is_shutdown():
            missing = [
                topic for topic in (self.ros_topic, self.ros_camera_info_topic)
                if not self.topic_exists(topic)
            ]
            if not missing:
                return
            if time.monotonic() >= deadline:
                break
            rospy.logwarn_throttle(5.0, "Waiting for camera topics: %s", ", ".join(missing))
            rospy.sleep(0.2)
        raise ValueError("Camera topics not available after %.1f seconds: %s" %
                         (timeout, ", ".join(missing)))

    async def signaling(self, websocket, pc):
        while True:
            message = await websocket.recv()
            print(f"Received message: {message}")

            if message == BYE:
                print("Received BYE, stopping")
                await pc.close()
                break
            else:
                msg = json.loads(message)

                if 'candidate' in msg:
                    candidate_parts = msg['candidate'].split(' ')
                    print("Split candidate parts:")
                    for index, part in enumerate(candidate_parts):
                        print(f"Part {index}: {part}")

                    ice_candidate = RTCIceCandidate(
                        foundation=candidate_parts[0].split(':')[1],
                        component=int(candidate_parts[1]),
                        protocol=candidate_parts[2],
                        priority=int(candidate_parts[3]),
                        ip=candidate_parts[4],
                        port=int(candidate_parts[5]),
                        type=candidate_parts[7],
                        sdpMid=msg.get('sdpMid', None),
                        sdpMLineIndex=msg.get('sdpMLineIndex', None)
                    )

                    await pc.addIceCandidate(ice_candidate)
                    print("after add ice candidate ")
                elif 'sdp' in msg:
                    if msg['type'] == 2 or msg['type'] == '2':
                        msg['type'] = "answer"
                    
                    desc = RTCSessionDescription(sdp=msg['sdp'], type=msg['type'])
                    await pc.setRemoteDescription(desc)

                    if desc.type == "offer":
                        await pc.setLocalDescription(await pc.createAnswer())
                        await websocket.send(json.dumps({
                            'sdp': pc.localDescription.sdp,
                            'type': pc.localDescription.type
                        }))

    async def create_offer(self, websocket, pc):
        print(f"create_offer called at {datetime.now().isoformat()}")
        await pc.setLocalDescription(await pc.createOffer())
        await websocket.send(json.dumps({
            'sdp': pc.localDescription.sdp,
            'type': pc.localDescription.type
        }))

    class ImageVideoTrack(VideoStreamTrack):
        """Emit a frame only when the ROS side actually delivered a new one.

        The previous version used aiortc's next_timestamp(), which advances pts by
        a fixed 1/30 s per call regardless of when frames arrive.  Whenever the
        encoder cannot keep up with 30 fps, pts therefore advances slower than the
        wall clock and the receiver's playout point drifts further and further
        behind -- latency that accumulates and never recovers.  Deriving pts from
        elapsed wall-clock time removes that drift, and waiting for a new frame
        stops us from re-encoding the same image several times.
        """

        MAX_FRAME_WAIT = 1.0  # resend the last frame if the source stalls, to keep the connection alive

        def __init__(self, client):
            super().__init__()
            self.client = client
            self._start = None
            self._last_seq = -1

        async def recv(self):
            deadline = time.time() + self.MAX_FRAME_WAIT
            while self.client.frame_seq == self._last_seq and time.time() < deadline:
                await asyncio.sleep(0.002)
            self._last_seq = self.client.frame_seq

            while self.client.latest_frame is None:
                await asyncio.sleep(0.01)
            img = self.client.latest_frame

            now = time.time()
            if self._start is None:
                self._start = now
            frame = av.VideoFrame.from_ndarray(img, format="bgr24")
            frame.pts = int((now - self._start) * VIDEO_CLOCK_RATE)
            frame.time_base = VIDEO_TIME_BASE
            return frame

    def camera_info_callback(self,msg):
        self.width = msg.width
        self.height = msg.height
        # print(f"Camera resolution: {msg.width}x{msg.height}")

    def image_callback(self, ros_image):
        self.latest_frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        self.frame_seq += 1

    async def main(self):
        while not self.start_connect_webrtc_singal:
            # print("Wait Quest3 Connect for start video stream ")
            await asyncio.sleep(1)

        print("Start video stream...")

        uri = f"ws://{self.server_ip}:8765"

        self.pc = RTCPeerConnection()

        video_track = self.ImageVideoTrack(self)

        self.pc.addTrack(video_track)
        print("Video track has been added to the peer connection.")

        async with websockets.connect(uri) as websocket:
            await websocket.send("client1")
            await self.create_offer(websocket, self.pc)
            await self.signaling(websocket, self.pc)

        await self.pc.close()

    async def handle_ros(self):
        # rospy.init_node('image_to_webrtc', anonymous=True)
        # queue_size 를 안 주면 rospy 는 무한 큐라 지연이 누적되고 회복되지 않는다.
        # buff_size 기본값(64KB)은 720p 프레임(2.76MB) 하나에 소켓 read 를 수십 번 하게 만든다.
        rospy.Subscriber(self.ros_topic, Image, self.image_callback,
                         queue_size=1, buff_size=16 * 1024 * 1024, tcp_nodelay=True)
        rospy.Subscriber(self.ros_camera_info_topic, CameraInfo, self.camera_info_callback)

        while not rospy.is_shutdown():
            await asyncio.sleep(0.03)

    def handle_signal(self, signal, frame):
        print("Received Ctrl-C, stopping...")
        sys.exit(0)


    async def run_in_background(self):
        print(f"WebRTCVideoStreamClient started at {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        ros_task = asyncio.create_task(self.handle_ros())
        main_task = asyncio.create_task(self.main())
        await asyncio.gather(ros_task, main_task)

    def start(self):
        def run_event_loop_in_thread():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            try:
                loop.run_until_complete(self.run_in_background())
            except KeyboardInterrupt:
                print("Received Ctrl-C, stopping...")
            finally:
                loop.close()

        thread = threading.Thread(target=run_event_loop_in_thread, daemon=True)
        thread.start()
        print("WebRTC client started in background thread")
