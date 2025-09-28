# camera_plugin.py
import os
import time
import threading
import cv2
import numpy as np

from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QFrame
from python_qt_binding.QtGui import QPixmap, QImage, QFont
from python_qt_binding.QtCore import QTimer, Qt, Signal, QObject

from rqt_gui_py.plugin import Plugin

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class RosWorker(Node):
    """rclpy node running in background thread — subscribes to camera topic and publishes to /image"""
    def __init__(self, image_topic="/camera/image_raw"):
        super().__init__('rdj2025_rqt_camera_worker')
        self.bridge = CvBridge()
        self._lock = threading.Lock()
        self.latest_frame = None

        # subscribe to camera feed for display
        self.image_sub = self.create_subscription(Image, image_topic, self._image_cb, 10)

        # publisher for snapshots/record frames aimed at potato detection node
        self.snapshot_pub = self.create_publisher(Image, "/image", 10)

    def _image_cb(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warning(f"cv_bridge error: {e}")
            return
        with self._lock:
            self.latest_frame = cv_img

    def get_latest_frame(self):
        with self._lock:
            if self.latest_frame is None:
                return None
            return self.latest_frame.copy()

    def publish_frame_to_image_topic(self, cv_frame):
        try:
            msg = self.bridge.cv2_to_imgmsg(cv_frame, encoding="bgr8")
            self.snapshot_pub.publish(msg)
            self.get_logger().info("Published frame to /image")
        except Exception as e:
            self.get_logger().error(f"Publish failed: {e}")

class CameraPlugin(Plugin):
    def __init__(self, context):
        super(CameraPlugin, self).__init__(context)
        self.setObjectName('CameraPlugin')

        # init rclpy if not already done
        if not rclpy.ok():
            rclpy.init()

        # GUI widget
        self._widget = QWidget()
        self._widget.setWindowTitle('RDJ Camera Panel')
        vbox = QVBoxLayout()
        self._widget.setLayout(vbox)

        # controls
        hbox = QHBoxLayout()
        self.btn_stream = QPushButton('Start Stream')
        self.btn_snapshot = QPushButton('Snapshot')
        self.btn_record = QPushButton('Start Recording')
        hbox.addWidget(self.btn_stream)
        hbox.addWidget(self.btn_snapshot)
        hbox.addWidget(self.btn_record)
        vbox.addLayout(hbox)

        # display label
        self.image_label = QLabel()
        self.image_label.setFixedSize(640, 480)
        self.image_label.setFrameStyle(QFrame.Box | QFrame.Plain)
        vbox.addWidget(self.image_label, alignment=Qt.AlignCenter)

        # status label
        self.status_label = QLabel("Status: idle")
        self.status_label.setFont(QFont("Sans", 10))
        vbox.addWidget(self.status_label, alignment=Qt.AlignCenter)

        context.add_widget(self._widget)

        # Ros worker
        self._worker = RosWorker(image_topic="/camera/image_raw")
        self._rclpy_thread = threading.Thread(target=self._spin_worker, daemon=True)
        self._rclpy_thread.start()

        # state
        self._streaming = False
        self._recording = False
        self._video_writer = None
        self._record_filename = None

        # connect signals
        self.btn_stream.clicked.connect(self._toggle_stream)
        self.btn_snapshot.clicked.connect(self._take_snapshot)
        self.btn_record.clicked.connect(self._toggle_record)

        # timer: refresh display ~10 FPS
        self._refresh_timer = QTimer(self._widget)
        self._refresh_timer.timeout.connect(self._refresh_display)
        self._refresh_timer.start(100)

    def _spin_worker(self):
        try:
            rclpy.spin(self._worker)
        except Exception:
            pass

    def _toggle_stream(self):
        if not self._streaming:
            self._streaming = True
            self.btn_stream.setText("Stop Stream")
            self.status_label.setText("Status: streaming")
        else:
            self._streaming = False
            self.btn_stream.setText("Start Stream")
            self.image_label.clear()
            self.status_label.setText("Status: idle")

    def _refresh_display(self):
        if not self._streaming:
            return

        frame = self._worker.get_latest_frame()
        if frame is None:
            return

        # if recording: write and publish this frame to /image
        if self._recording and self._video_writer is not None:
            try:
                self._video_writer.write(frame)
                # publish frame to /image for potato detection
                self._worker.publish_frame_to_image_topic(frame)
            except Exception as e:
                self._worker.get_logger().warning(f"Recording write/publish error: {e}")

        # draw frame to QLabel
        h, w, ch = frame.shape
        bytes_per_line = ch * w
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        qimg = QImage(rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pix = QPixmap.fromImage(qimg).scaled(self.image_label.size(), Qt.KeepAspectRatio)
        self.image_label.setPixmap(pix)

    def _take_snapshot(self):
        frame = self._worker.get_latest_frame()
        if frame is None:
            self.status_label.setText("Status: no frame")
            return

        # publish to /image and save locally
        self._worker.publish_frame_to_image_topic(frame)
        filename = f"snapshot_{int(time.time())}.jpg"
        cv2.imwrite(filename, frame)
        self.status_label.setText(f"Snapshot saved/published: {filename}")

    def _toggle_record(self):
        if not self._recording:
            frame = self._worker.get_latest_frame()
            if frame is None:
                self.status_label.setText("Status: no frame to start recording")
                return
            self._record_filename = f"recording_{int(time.time())}.mp4"
            h, w, _ = frame.shape
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self._video_writer = cv2.VideoWriter(self._record_filename, fourcc, 20.0, (w, h))
            self._recording = True
            self.btn_record.setText("Stop Recording")
            self.status_label.setText("Recording...")
        else:
            self._recording = False
            self.btn_record.setText("Start Recording")
            if self._video_writer:
                self._video_writer.release()
                self._video_writer = None
            self.status_label.setText(f"Recording saved: {self._record_filename}")

    def shutdown_plugin(self):
        if self._video_writer:
            self._video_writer.release()
        try:
            if self._worker:
                self._worker.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
