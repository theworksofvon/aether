# Aether Setup Guide

## Prerequisites

- Raspberry Pi 5 running Ubuntu Server 24.04
- GPS module connected via USB
- Repo cloned to `/home/<user>/aether`

## 1. Install Dependencies

```bash
sudo apt update
sudo apt install -y build-essential cmake git podman
```

## 2. Fix Package Conflicts (Ubuntu 24.04)

If you hit dependency errors with `build-essential`:

```bash
sudo apt install libbz2-1.0=1.0.8-5.1 --allow-downgrades
sudo apt install -y build-essential cmake git
```

## 3. Build the Container Image

```bash
cd ~/aether
podman build -t localhost/aether:latest .
```

## 4. Build ROS 2 Nodes

```bash
podman run -it --rm \
  --device /dev/ttyUSB0 \
  --privileged \
  -v ~/aether:/aether \
  localhost/aether bash

# Inside container
source /opt/ros/jazzy/setup.bash
colcon build
exit
```

## 5. Set Up Auto-Start on Boot

Load the image into root's storage (required for systemd):

```bash
podman save localhost/aether:latest -o /tmp/aether.tar
sudo podman load -i /tmp/aether.tar
```

Start a named container to generate the service from:

```bash
sudo chmod 666 /dev/ttyUSB0

podman run -d \
  --name aether \
  --device /dev/ttyUSB0 \
  --privileged \
  -v ~/aether:/aether \
  localhost/aether \
  /bin/bash -c "source /opt/ros/jazzy/setup.bash && source /aether/install/setup.bash && ros2 run gps_node gps_node"
```

Generate and install the systemd service:

```bash
podman generate systemd --new --name aether | sudo tee /etc/systemd/system/aether.service
```

Add GPS permission to the service file — insert before `ExecStart`:

```ini
ExecStartPre=/bin/chmod 666 /dev/ttyUSB0
```

Enable and start:

```bash
sudo systemctl daemon-reload
sudo systemctl enable aether.service
sudo systemctl start aether.service
```

## 6. Verify

```bash
sudo systemctl status aether.service
```

You should see GPS data streaming in the logs. Reboot to confirm auto-start works.

## Updating Code

```bash
# Edit source files
# Then rebuild inside container
podman run -it --rm \
  --device /dev/ttyUSB0 \
  --privileged \
  -v ~/aether:/aether \
  localhost/aether bash -c \
  "source /opt/ros/jazzy/setup.bash && colcon build"

# Restart service to pick up changes
sudo systemctl restart aether.service
```

## Troubleshooting

| Issue | Fix |
|-------|-----|
| GPS permission denied | `sudo chmod 666 /dev/ttyUSB0` |
| Container not found by systemd | `sudo podman load -i /tmp/aether.tar` |
| Service keeps restarting | `journalctl -xeu aether.service` |
| No GPS data | Check `ls /dev/ttyUSB*` and verify module is connected |
