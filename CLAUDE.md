# Aether — Claude Review Guidelines

Aether is an edge-autonomous drone platform. All code runs on a Raspberry Pi 5 (Ubuntu Server 24.04, armv8) with no internet connectivity during flight. There is no cloud backend, no authentication layer, and no database. The architecture is a ROS 2 Jazzy pub/sub mesh of isolated nodes, each deployed as its own Docker/Podman container.

## Stack

| Layer | Technology |
|-------|-----------|
| Robotics middleware | ROS 2 Jazzy |
| C++ nodes | C++20, ament_cmake, rclcpp |
| Python nodes | Python 3, ament_python, rclpy |
| Flight controller comms | MAVLink (pymavlink) |
| AI inference | YOLOv8 Nano (ultralytics), OpenCV |
| Containerization | Docker / Podman (base: `ros:jazzy-ros-base`) |
| Target hardware | Raspberry Pi 5, ArduPilot FC, NEO-6M GPS, ESP32 sensor hub, Pi Camera |

## Node Map

| Node | Language | Status | Key file |
|------|----------|--------|----------|
| `gps_node` | C++ | Working | `src/gps_node/src/gps_node.cpp` |
| `flight_node` | Python | In progress | `src/flight_node/flight_node/flight_node.py` |
| `sensor_node` | Python | In progress | `src/sensor_node/sensor_node/sensor_node.py` |
| `vision_node` | Python | In progress | `src/vision_node/vision_node/vision_node.py` |
| `autonomy_node` | C++ | Stub — intentionally incomplete | `src/autonomy_node/src/autonomy_node.cpp` |

**Do not flag `autonomy_node` for syntax errors or incomplete implementation — it is a known stub.**

---

## Code Style Conventions

### C++

- **Classes and structs:** PascalCase (`GpsNode`, `GGA`, `SentenceType`)
- **Functions:** snake_case (`nmea_to_decimal`, `open_serial`, `strip_checksum`)
- **Private member variables:** trailing underscore (`fd_`, `fix_publisher_`, `timer_`)
- **Indentation:** 4 spaces, no tabs
- **Includes:** standard library first, then ROS headers, then third-party
- **Enums:** `enum class` with PascalCase values
- Use `std::string`, `std::vector`, and modern C++20 idioms — avoid C-style casts and raw pointers where possible

### Python

- **Classes:** PascalCase, always extend `rclpy.node.Node`
- **Methods and variables:** snake_case
- **Publisher fields:** trailing underscore convention (`publisher_`, `gps_publisher_`)
- Follow PEP 8 for spacing and line length
- f-strings preferred for string formatting

### Shared

- **Logging:** Always use the ROS logger — never `print()` or `std::cout` in production code
  - C++: `RCLCPP_INFO(this->get_logger(), "[TAG] message %s", value)`
  - Python: `self.get_logger().info(f"message {value}")`
- **Comments:** Only when the WHY is non-obvious. No restating what the code does.
- No docstrings required, but public-facing functions benefit from a one-liner.

---

## ROS 2 Architecture Patterns

- All nodes inherit from `rclcpp::Node` (C++) or `rclpy.node.Node` (Python)
- Node initialization happens in the constructor: publishers, subscriptions, and timers
- Use `create_timer(interval_seconds, callback)` for all periodic work — no `sleep()` loops
- Use standard ROS message types (`sensor_msgs`, `geometry_msgs`, `vision_msgs`) until custom messages are explicitly added
- Communication is pub/sub only — no direct inter-node calls or service calls (yet)
- Each node is independently containerized; do not add cross-node imports

---

## Error Handling

### Python nodes
- Wrap hardware init in try/except; log failures with `self.get_logger().error()`
- Hardware unavailability should set the resource to `None` and skip gracefully in the timer callback
- Use `self.get_logger().warn()` for recoverable issues, `.error()` for failures

### C++ nodes
- Check return values for all system calls (`open`, `tcgetattr`, `read`)
- Use early returns with `std::cerr` for fatal init failures
- Use `RCLCPP_WARN` / `RCLCPP_ERROR` macros for runtime issues

**Flag:** silent exception swallowing (`except: pass`), missing `if fd < 0` checks, or any hardware call without error handling.

---

## Common Pitfalls to Flag

- **Blocking I/O in timer callbacks** — serial reads or MAVLink waits without a timeout will freeze the ROS executor. Use non-blocking reads or timeouts.
- **Missing hardware init error handling** — serial port open, MAVLink heartbeat wait, GPIO init must all handle failure.
- **Hardcoded device paths** — use environment variables with a sensible default (see `GPS_PORT` pattern in `gps_node.cpp`).
- **Unvalidated external data** — NMEA sentences and MAVLink messages are untrusted input from hardware; validate fields before parsing or indexing into them.
- **Missing model file handling** in `vision_node` — the YOLOv8 model (`yolov8n.pt`) is downloaded at build time; flag code that doesn't handle a missing or corrupt model file.
- **Blocking MAVLink calls** — `wait_heartbeat()` and similar calls must always have a `timeout` argument.
- **`std::cout` / `print()` usage** — use the ROS logger instead.

---

## Security Considerations

This is an offline, embedded platform — there is no network-facing attack surface during normal operation. Security concerns are physical and protocol-layer:

- **Serial / NMEA input validation:** Treat all data from `/dev/ttyUSB0` as untrusted. Check sentence type, field count, and checksum before parsing.
- **MAVLink message validation:** Check message ID and field ranges before acting on received commands.
- **Device path injection:** If device paths are ever derived from environment variables or config files, validate them before passing to `open()`.
- **Resource exhaustion:** Timer callbacks that block indefinitely can starve other nodes. All I/O must have timeouts.
- **Container privileges:** Docker/Podman containers need device access (`--device`); flag any new `--privileged` flags or overly broad device mounts.

---

## Testing

There is no automated test suite yet. When reviewing PRs:

- Flag hardware logic additions with no accompanying manual test notes in the PR description
- When tests are eventually added: C++ nodes will use `colcon test` (ament_cmake test targets), Python nodes will use `pytest`
- Integration testing is done manually via `ros2 topic echo` and `ros2 node info`

---

## Linting and Formatting

No linting configs exist in this repo yet. Flag obvious violations as style suggestions (not blocking issues):

- **Python:** PEP 8 — line length, whitespace, naming
- **C++:** clang-format style — consistent brace placement, spacing

---

## Dockerfile / Container Changes

- Base image is `ros:jazzy-ros-base` for all nodes
- Target architecture is **armv8 (Raspberry Pi 5)** — flag any dependency that is x86-only or lacks an arm64 package
- Each node has its own Dockerfile (`Dockerfile.gps`, `Dockerfile.flight`, `Dockerfile.sensor`, `Dockerfile.vision`)
- Avoid adding packages that pull in heavy GUI or desktop dependencies (this is a headless server image)
- `colcon build` is run inside the container; flag changes to `CMakeLists.txt` or `package.xml` that could break the build
