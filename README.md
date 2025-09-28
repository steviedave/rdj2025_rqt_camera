# RDJ2025 RQT Camera Plugin

This package provides an `rqt` plugin to display a live camera feed, take snapshots, and record video while publishing frames to a ROS 2 topic (`/image`). The `/image` topic is used by the potato disease detection node or any other consumer node.

---

## Features

* **Start Stream**: Displays frames from `/camera/image_raw`.
* **Snapshot**: Publishes the current frame to `/image` and saves it as `snapshot_<timestamp>.jpg`.
* **Record**: Records video locally to `recording_<timestamp>.mp4` and continuously publishes frames to `/image`.

---

## Requirements

* ROS 2 Humble (tested on Raspberry Pi 4)
* Python 3
* Dependencies:

  ```bash
  sudo apt install -y python3-opencv python3-pyqt5 \
    ros-humble-rqt-gui ros-humble-rqt-gui-py ros-humble-python-qt-binding \
    ros-humble-cv-bridge
  ```

---

## Build Instructions

Clone this package into your ROS 2 workspace (`~/ros2_ws/src`):

```bash
cd ~/ros2_ws/src
git clone <your_repo_url> rdj2025_rqt_camera
```

Build the package:

```bash
cd ~/ros2_ws
colcon build --packages-select rdj2025_rqt_camera
source install/setup.bash
```

---

## Running

### 1. Start the camera publisher

Publishes frames to `/camera/image_raw`:

```bash
python3 ~/ros2_ws/src/rdj2025_rqt_camera/rdj2025_rqt_camera/camera_publisher.py
```

### 2. Start the potato disease detection node

(Replace with your package/node names. This node must subscribe to `/image`.)

```bash
ros2 run rdj2025_potato_disease_detection potato_disease_detection_node
```

### 3. Start `rqt` and open the plugin

Launch `rqt`:

```bash
rqt
```

In the menu, go to:

```
Plugins → Python → RDJ Camera Plugin
```

Or run standalone:

```bash
rqt --standalone rdj2025_rqt_camera.camera_plugin
```

---

## Usage

1. **Start Stream**: Displays live camera feed from `/camera/image_raw`.
2. **Snapshot**:

   * Publishes the current frame to `/image` (for potato node).
   * Saves an image file (`snapshot_<timestamp>.jpg`).
3. **Record**:

   * Toggles video recording.
   * While recording: saves frames to a `.mp4` and publishes each frame to `/image`.
   * Click again to stop recording.

---

## Topics

* **Published**

  * `/image` (`sensor_msgs/Image`): snapshot/record frames.

* **Subscribed**

  * `/camera/image_raw` (`sensor_msgs/Image`): input camera feed.

---

## Notes

* Ensure your camera device is accessible (`/dev/video0`). Add your user to the `video` group if needed.
* If your camera publishes to a topic other than `/camera/image_raw`, update the `image_topic` argument in `camera_plugin.py`.
* CPU usage on Raspberry Pi increases during recording; adjust timer refresh interval for performance.

---

## Example Workflow

1. Start camera publisher.
2. Open the rqt plugin.
3. Click **Start Stream** to see live video.
4. Click **Snapshot** → potato disease node receives one frame.
5. Click **Start Recording** → potato disease node receives continuous frames.
6. Inference results appear on `/inference_result` (or your chosen topic).
7. A decision node can listen and publish `/servo_control` for actuation.

