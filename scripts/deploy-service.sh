#!/bin/bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SERVICE_SOURCE="$REPO_DIR/systemd/aether.service"
SERVICE_TARGET="/etc/systemd/system/aether.service"

echo "Deploying aether.service from $SERVICE_SOURCE"
sudo install -m 0644 "$SERVICE_SOURCE" "$SERVICE_TARGET"
sudo systemctl daemon-reload
sudo systemctl enable aether.service
echo "Service installed and enabled"
echo "Start with: sudo systemctl start aether"
echo "Stop with: sudo systemctl stop aether"
echo "Status with: sudo systemctl status aether"
