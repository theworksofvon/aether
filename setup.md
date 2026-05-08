# Aether Setup Guide

## Prerequisites

- Raspberry Pi 5 running Ubuntu Server 24.04
- USB-UART adapter connecting the Pi to the Matek flight controller
- Repo cloned to `/home/<user>/aether`

## 1. Install Dependencies

```bash
sudo apt update
sudo apt install -y build-essential cmake git podman python3-pip
```

## 2. Fix Package Conflicts (Ubuntu 24.04)

If you hit dependency errors with `build-essential`:

```bash
sudo apt install libbz2-1.0=1.0.8-5.1 --allow-downgrades
sudo apt install -y build-essential cmake git
```

## 3. Configure the Runtime

```bash
cd ~/aether
cp .env.example .env
```

Update `.env` for your hardware, especially:

- `AETHER_DRONE_ID`
- `AETHER_FLIGHT_DEVICE`
- `AETHER_FLIGHT_BAUD_RATE`
- `AETHER_FLIGHT_MODE_CONFIRM_TIMEOUT_S`
- `AETHER_COMMAND_AUTH_TOKEN`

For a USB-UART bridge to the Matek, `AETHER_FLIGHT_DEVICE` will typically be something like `/dev/ttyUSB0`.
Set `AETHER_COMMAND_AUTH_TOKEN` to a secret value shared by the `edge` and `flight` services before using the stack outside local development.

## 4. Build the Container Stack

```bash
./scripts/build-containers.sh
```

## 5. Set Up Auto-Start on Boot

Install the systemd unit from the repo into `/etc/systemd/system/aether.service`:

```bash
sudo ./scripts/deploy-service.sh
sudo systemctl start aether
```

`deploy-service.sh` renders the unit with the repo's current absolute path, so you do not need to edit the service file if the checkout lives somewhere other than a hardcoded home directory.

## 6. Verify

```bash
sudo systemctl status aether
podman compose ps
```

You should see the `edge` and `flight` services running. Reboot to confirm auto-start works.

Optional services can be started with the compose profile:

```bash
podman compose --profile optional up -d
```

## 7. Local Development

For host-side development, use `uv` and only install the groups you actually need:

```bash
uv venv
source .venv/bin/activate
uv sync --group edge --group flight
```

Add `--group vision` if you need the vision stack locally.
The `sensor` group is better treated as Pi-targeted unless you specifically need it on your host machine.

## Updating Code

```bash
# Rebuild the stack after source changes
./scripts/build-containers.sh

# Restart the runtime
sudo systemctl restart aether
```

## Troubleshooting

| Issue | Fix |
|-------|-----|
| Flight controller not detected | Check `ls /dev/ttyUSB*` and verify the USB-UART adapter is connected |
| Service failed to start | `journalctl -xeu aether.service` |
| Containers not running | `podman compose ps` |
| Local `uv sync` fails on sensor deps | Skip `--group sensor` on non-Pi machines unless you have the required system build tools |
