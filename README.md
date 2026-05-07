# aether 🚁

> Edge-autonomous drone platform with ROS 2, MAVLink, and onboard AI inference. Built from scratch in C++.

![Status](https://img.shields.io/badge/status-active%20development-orange?style=flat-square)
![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue?style=flat-square&logo=ros)
![C++](https://img.shields.io/badge/C++-20-00599C?style=flat-square&logo=c%2B%2B&logoColor=white)
![Platform](https://img.shields.io/badge/platform-Raspberry%20Pi%205-red?style=flat-square)
![License](https://img.shields.io/badge/license-MIT-green?style=flat-square)

---

**⚠️ This project is actively being built. Not production ready. Watch the repo to follow along.**

---

## What is this?

Aether is a full-stack autonomous drone platform built from the ground up. The goal is a drone that can navigate, sense its environment, make decisions, and operate completely offline — no cloud, no internet, no dependency on external services.

Everything runs on a Raspberry Pi 5 mounted on the drone. C++ all the way down.

For multi-drone use, this repo is the onboard edge runtime that runs on each aircraft. Fleet aggregation, operator UX, and mission coordination live above it; the local drone stack remains responsible for hardware access, local autonomy, and safe behavior when the coordinator link drops.

## Architecture

```
┌─────────────────────────────────────────────────────┐
│                  Raspberry Pi 5                      │
│                                                     │
│  ┌──────────┐  ┌──────────────────┐                │
│  │ sensor   │  │  vision_node     │                │
│  │ _node    │  │  (YOLOv8 Nano)   │                │
│  └────┬─────┘  └────────┬─────────┘                │
│       │                 │                          │
│       └─────────────────┘                          │
│                     │                               │
│              ROS 2 Graph                            │
│                     │                               │
│            ┌────────┴────────┐                      │
│            │  flight_node    │                      │
│            │  (MAVLink + GPS)│                      │
│            └────────┬────────┘                      │
└─────────────────────┼───────────────────────────────┘
                      │ USB-UART
              ┌───────┴────────┐
              │ Flight         │
              │ Controller     │
              │ (ArduPilot)    │
              └───────┬────────┘
                      │
              ESCs → Motors → Props
```

## Stack

| Layer | Technology |
|-------|-----------|
| Language | C++20 |
| Robotics Middleware | ROS 2 Jazzy |
| Flight Controller Firmware | ArduPilot |
| Flight Controller Communication | MAVLink |
| Companion Computer | Raspberry Pi 5 |
| Sensor Hub | ESP32 (UART → Pi) |
| Container Runtime | Podman |
| AI Inference | YOLOv8 Nano / TFLite |

## Nodes

| Node | Status | Description |
|------|--------|-------------|
| `sensor_node` | 🔨 Planned | Reads ESP32 sensor data (temp, humidity, ultrasonic, sound) |
| `vision_node` | 🔨 Planned | Pi Camera + YOLOv8 object detection |
| `flight_node` | 🔨 Planned | MAVLink bridge to ArduPilot flight controller, including FC-sourced GPS |
| `edge_node` | ✅ Working | Fleet-facing command ingress and telemetry/status egress for one drone |

## ROS 2 Topics

| Topic | Message Type | Publisher |
|-------|-------------|-----------|
| `/sensors` | TBD | sensor_node |
| `/detections` | TBD | vision_node |

## Multi-Drone Edge Runtime

Each drone now runs the same Aether stack with a unique `drone_id`.
Topic names are namespaced per aircraft so multiple drones can coexist on one network without collisions.

Examples for drone `AE-01`:

| Topic | Message Type | Purpose |
|-------|-------------|---------|
| `/AE-01/flight/gps` | `sensor_msgs/NavSatFix` | MAVLink-derived flight GPS |
| `/AE-01/flight/gps_status` | `sensor_msgs/NavSatStatus` | MAVLink-derived GPS fix status |
| `/AE-01/flight/attitude` | `geometry_msgs/Vector3` | Roll, pitch, yaw |
| `/AE-01/flight/battery` | `sensor_msgs/BatteryState` | Battery state |
| `/AE-01/flight/mode` | `std_msgs/String` | Current flight mode |
| `/AE-01/vision/detections` | `vision_msgs/Detection2DArray` | Vision detections |
| `/AE-01/sensors/dht11` | `std_msgs/Float32MultiArray` | DHT11 sensor readings |
| `/AE-01/flight/commands` | `std_msgs/String` | Flight commands routed from fleet control |
| `/AE-01/autonomy/commands` | `std_msgs/String` | Autonomy commands routed from fleet control |
| `/AE-01/vision/commands` | `std_msgs/String` | Vision commands routed from fleet control |
| `/AE-01/mission/commands` | `std_msgs/String` | Mission commands routed from fleet control |
| `/AE-01/system/commands` | `std_msgs/String` | Edge/system commands routed from fleet control |
| `/AE-01/flight/events` | `std_msgs/String` | Flight node completion/result events |
| `/AE-01/autonomy/events` | `std_msgs/String` | Autonomy node completion/result events |
| `/AE-01/vision/events` | `std_msgs/String` | Vision node completion/result events |
| `/AE-01/mission/events` | `std_msgs/String` | Mission node completion/result events |
| `/AE-01/system/events` | `std_msgs/String` | System node completion/result events |
| `/AE-01/edge/acks` | `std_msgs/String` | Local edge acknowledgement and completion stream |
| `/fleet/commands` | `std_msgs/String` | Fleet command ingress |
| `/fleet/acks` | `std_msgs/String` | Fleet acknowledgement egress |
| `/fleet/telemetry` | `std_msgs/String` | Fleet telemetry summary egress |

`edge_node` now acts as a typed router: it accepts a fleet command, checks whether the target matches this drone instance, resolves the responsible subsystem, publishes the command onto that subsystem's command topic, and relays completion events from subsystem event topics back to local and fleet acknowledgement streams.

The fleet-facing and route topics currently use JSON payloads in `std_msgs/String` so the edge contract can evolve without introducing a custom message package yet.

## Configuration

Runtime configuration now has a shared Python source of truth in [config.py](/home/von/dev/aether/config.py).
That file groups environment-backed settings for:

- drone identity and groups
- flight controller serial link
- sensor settings
- vision settings
- edge/fleet topics and timing

Each Python node reads from the shared config model first, then exposes ROS parameters on top of those validated defaults.

## Deployment

Aether runs as a split edge stack rather than one monolith. Each node can fail and restart independently, which keeps non-essential failures from blocking the whole drone runtime.

- `compose.yaml` defines the containerized stack
- `systemd/aether.service` manages stack startup at boot
- `scripts/build-containers.sh` builds the containers
- `scripts/deploy-service.sh` installs and enables the systemd unit

Service model:

- `edge` and `flight` are core services
- `sensor` and `vision` are optional services under the `optional` compose profile
- container restart policy handles per-node recovery
- systemd manages stack lifecycle, not individual node restart loops

Typical operator flow:

```bash
cp .env.example .env
./scripts/build-containers.sh
sudo ./scripts/deploy-service.sh
sudo systemctl start aether
```

To include optional services at runtime:

```bash
podman compose --profile optional up -d
```

## Hardware

| Component | Part |
|-----------|------|
| Frame | 450mm (3D printed, Bambu P1S) |
| Motors | 2212 1000KV x4 |
| ESCs | 30A BLHeli_S x4 |
| Flight Controller | Matek F405-Wing V2 (ArduPilot) |
| Companion Computer | Raspberry Pi 5 |
| GPS | Connected to Matek F405-Wing V2 |
| Sensor Hub | ESP32 |
| Battery | 3S 2200mAh LiPo |
| Radio | FlySky FS-i6X |

## Getting Started

### Prerequisites
- Raspberry Pi 5 running Ubuntu Server 24.04
- Podman installed
- USB-UART adapter connecting the Pi to the Matek flight controller

### Setup

```bash
git clone https://github.com/theworksofvon/aether
cd aether
podman build -t aether .
```

## Roadmap

- [x] MAVLink-based GPS telemetry from the flight controller
- [x] ROS 2 Jazzy containerized environment
- [x] GPS node publishing live position data
- [ ] Flight controller MAVLink integration
- [ ] ESP32 sensor node
- [ ] ArduPilot waypoint navigation
- [ ] Pi Camera + AI vision node
- [ ] Autonomous survey patterns
- [ ] Ground station React dashboard
- [ ] Multi-drone swarm coordination
