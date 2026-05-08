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

`flight_node` now consumes routed JSON commands from `/{drone_id}/flight/commands` and publishes command result events to `/{drone_id}/flight/events`. The current flight command surface is intentionally small: `ping`, `health_check`, `set_mode`, `hold`, and `rtl` are supported; unsupported commands fail explicitly with a structured `failed` event. Mode-change commands only publish `completed` after the requested flight mode is actually observed on heartbeat telemetry.

## Configuration

Runtime configuration now has a shared Python source of truth in [config.py](/home/von/dev/aether/config.py).
That file groups environment-backed settings for:

- drone identity and groups
- flight controller serial link
- sensor settings
- vision settings
- edge/fleet topics and timing

Each Python node reads from the shared config model first, then exposes ROS parameters on top of those validated defaults.

The internal `common` package is not its own long-running service. It is a small Python helper library that gets built into only the node images that import it, so it does not make `sensor_node` or `vision_node` depend on `edge_node` or `flight_node` at runtime.

The `edge` and `flight` services also share an HMAC token used to sign routed commands and completion events. That token has no built-in usable default at runtime: `edge_node` and `flight_node` will refuse to start if it is missing or still set to the example placeholder value.

## Python Dependencies

Python dependencies are managed at the repo root with `uv` in [pyproject.toml](/home/von/dev/aether/pyproject.toml).
Dependencies are split by runtime so each container installs only what it needs:

- `edge`
- `flight`
- `sensor`
- `vision`

The Dockerfiles install only their own dependency group plus the shared config dependencies, which keeps `vision` heavy without forcing that weight into `edge` or `flight`.

## Development Modes

Do not treat your local workstation as if it has to perfectly mirror the Raspberry Pi runtime.
Aether has two valid development environments:

### Local host development

Use this for:

- code editing
- Python validation
- edge and flight logic
- fast iteration without hardware

Typical local setup:

```bash
uv venv
source .venv/bin/activate
uv sync --group edge --group flight
```

If you need vision locally:

```bash
uv sync --group edge --group flight --group vision
```

You usually should not install the `sensor` group on a non-Pi machine unless you are specifically working on that node and have the system build dependencies it needs.

### Pi and container runtime validation

Use this for:

- MAVLink serial integration
- camera access
- GPIO access
- startup and restart behavior
- anything that depends on the real drone hardware path

This is the source of truth for runtime behavior.
Local `uv` environments are for development speed; Podman containers on the target device are for integration truth.

## Deployment

Aether runs as a split edge stack rather than one monolith. Each node can fail and restart independently, which keeps non-essential failures from blocking the whole drone runtime.

- `compose.yaml` defines the containerized stack
- `systemd/aether.service` is a template for boot startup
- `scripts/build-containers.sh` builds the containers
- `scripts/deploy-service.sh` renders the service with the current repo path, then installs and enables the systemd unit

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
cp .env.example .env
./scripts/build-containers.sh
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
