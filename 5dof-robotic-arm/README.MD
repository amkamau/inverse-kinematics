# 5-DOF Robotic Arm — Setup

## Prerequisites

- Ubuntu with ROS 2 Jazzy installed
- Python 3 and `colcon`
- Git
- Required ROS 2 visualization and joint-state packages

## 1. Install dependencies

Update package lists:

```bash
sudo apt update
```

Install the required ROS 2 packages:

```bash
sudo apt install -y \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-xacro \
  ros-jazzy-rviz2 \
  python3-colcon-common-extensions \
  python3-rosdep
```

Initialize `rosdep` if it has not already been initialized:

```bash
sudo rosdep init
rosdep update
```

If `rosdep init` reports that it is already initialized, continue with `rosdep update`.

## 2. Build the workspace

From the project directory:

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

## 3. Launch the robot visualization

```bash
ros2 launch five_dof_arm launch.py
```

## Notes

- Source `/opt/ros/jazzy/setup.bash` in each new terminal.
- Source `ros2_ws/install/setup.bash` after building the workspace.
- Rebuild with `colcon build` after making relevant source or configuration changes.
- Do not commit generated `build/`, `install/`, or `log/` directories to Git.





## Ubuntu Setup: Hiwonder xArm USB Connection (ROS 2 Jazzy)

This section configures USB access for the Hiwonder xArm controller and prepares the ROS 2 Python environment on a fresh Ubuntu installation, including Raspberry Pi 5 running Ubuntu.

### 1. Install prerequisites

Ensure ROS 2 Jazzy and the ROS 2 workspace dependencies are installed. Install the required system tools:

```bash
sudo apt update
sudo apt install -y python3-venv python3-pip udev
```

From the project root, create the virtual environment if it does not already exist. Use `--system-site-packages` so the environment can access ROS 2 Python packages installed through APT:

```bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python3 -m pip install hidapi
```

### 2. Configure USB permissions

The xArm controller identifies as USB vendor `0483`, product `5750`. The Python `hidapi` package may use the **libusb backend**, which needs write access to the USB device node.

Create a targeted udev rule:

```bash
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="0483", ATTR{idProduct}=="5750", MODE="0660", GROUP="plugdev"' \
  | sudo tee /etc/udev/rules.d/99-hiwonder-usb.rules
```

Reload udev rules:

```bash
sudo udevadm control --reload-rules
```

Ensure the current user belongs to `plugdev`:

```bash
sudo usermod -aG plugdev "$USER"
```

If group membership was newly added, log out and log back in for it to take effect.

Unplug and reconnect the xArm USB cable. Verify the device is detected:

```bash
lsusb -d 0483:5750
```

Check the USB device permissions using the bus and device numbers reported by `lsusb`. For example, if the device appears as Bus `004`, Device `002`:

```bash
ls -l /dev/bus/usb/004/002
```

The device should belong to group `plugdev` and have group read/write permissions, such as `crw-rw----`.

### 3. Verify HID access

Activate the project's virtual environment and test whether Python can open the controller:

```bash
python3 -c "import hid; d=hid.device(); d.open(0x0483, 0x5750); print('HID opened successfully'); d.close()"
```

Expected output:

```text
HID opened successfully
```

If this test fails, check the USB device permissions, current group membership, and whether the controller is connected.

### 4. Build and launch the ROS 2 workspace

From the project root:

```bash
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate

cd ros2_ws
colcon build --packages-select five_dof_arm --symlink-install
source install/setup.bash

ros2 launch five_dof_arm launch.py
```

**Important:** Source `install/setup.bash` in every new terminal before launching the package. Otherwise, ROS 2 may report:

```text
Package 'five_dof_arm' not found
```

If the workspace has already been built and the `install` directory exists, rebuilding is generally unnecessary unless the source code or dependencies have changed.

### 5. Troubleshooting

| Error | Possible cause / action |
|---|---|
| `OSError: open failed` | Check the targeted USB udev rule, reconnect the controller, and verify permissions on `/dev/bus/usb/...`. |
| `Package 'five_dof_arm' not found` | Source `/opt/ros/jazzy/setup.bash`, then source `ros2_ws/install/setup.bash`. |
| `ModuleNotFoundError: No module named 'hid'` | Activate the project `.venv` and install `hidapi` in that environment. |
| `Arm read failed: list index out of range` | USB opening may already be working. Investigate the controller response and the parsing logic in `xarm_state_node.py`; this error is not resolved by the USB permissions rule alone. |

**Security note:** The udev rule grants access only to USB devices matching vendor `0483` and product `5750`, for members of `plugdev`. Do not run the ROS 2 nodes as root merely to bypass device permissions.