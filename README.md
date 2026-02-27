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

## Architecture

```
┌─────────────────────────────────────────────────────┐
│                  Raspberry Pi 5                      │
│                                                     │
│  ┌──────────┐  ┌──────────┐  ┌──────────────────┐  │
│  │ gps_node │  │ sensor   │  │  vision_node     │  │
│  │          │  │ _node    │  │  (YOLOv8 Nano)   │  │
│  └────┬─────┘  └────┬─────┘  └────────┬─────────┘  │
│       │             │                 │             │
│       └─────────────┴─────────────────┘             │
│                     │                               │
│              ROS 2 Graph                            │
│                     │                               │
│            ┌────────┴────────┐                      │
│            │  flight_node    │                      │
│            │  (MAVLink)      │                      │
│            └────────┬────────┘                      │
└─────────────────────┼───────────────────────────────┘
                      │ UART
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
| `gps_node` | ✅ Working | Parses NMEA sentences from NEO-6M, publishes to `/gps/fix` |
| `sensor_node` | 🔨 Planned | Reads ESP32 sensor data (temp, humidity, ultrasonic, sound) |
| `vision_node` | 🔨 Planned | Pi Camera + YOLOv8 object detection |
| `flight_node` | 🔨 Planned | MAVLink bridge to ArduPilot flight controller |

## ROS 2 Topics

| Topic | Message Type | Publisher |
|-------|-------------|-----------|
| `/gps/fix` | `sensor_msgs/NavSatFix` | gps_node |
| `/gps/status` | `sensor_msgs/NavSatStatus` | gps_node |
| `/sensors` | TBD | sensor_node |
| `/detections` | TBD | vision_node |

## Hardware

| Component | Part |
|-----------|------|
| Frame | 450mm (3D printed, Bambu P1S) |
| Motors | 2212 1000KV x4 |
| ESCs | 30A BLHeli_S x4 |
| Flight Controller | Matek F405-Wing V2 (ArduPilot) |
| Companion Computer | Raspberry Pi 5 |
| GPS | NEO-6M |
| Sensor Hub | ESP32 |
| Battery | 3S 2200mAh LiPo |
| Radio | FlySky FS-i6X |

## Getting Started

### Prerequisites
- Raspberry Pi 5 running Ubuntu Server 24.04
- Podman installed
- GPS module connected to `/dev/ttyUSB0`

### Setup

```bash
git clone https://github.com/theworksofvon/aether
cd aether
podman build -t aether .
```

## Roadmap

- [x] GPS NMEA parser in C++
- [x] ROS 2 Jazzy containerized environment
- [x] GPS node publishing live position data
- [ ] Flight controller MAVLink integration
- [ ] ESP32 sensor node
- [ ] ArduPilot waypoint navigation
- [ ] Pi Camera + AI vision node
- [ ] Autonomous survey patterns
- [ ] Ground station React dashboard
- [ ] Multi-drone swarm coordination
