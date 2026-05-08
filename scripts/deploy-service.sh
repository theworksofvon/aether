#!/bin/bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SERVICE_SOURCE="$REPO_DIR/systemd/aether.service"
SERVICE_TARGET="/etc/systemd/system/aether.service"
TMP_SERVICE="$(mktemp)"

cleanup() {
  rm -f "$TMP_SERVICE"
}

trap cleanup EXIT

echo "Deploying aether.service from $SERVICE_SOURCE"
sed "s|__AETHER_REPO_DIR__|$REPO_DIR|g" "$SERVICE_SOURCE" > "$TMP_SERVICE"
sudo install -m 0644 "$TMP_SERVICE" "$SERVICE_TARGET"
sudo systemctl daemon-reload
sudo systemctl enable aether.service
echo "Service installed and enabled"
echo "Start with: sudo systemctl start aether"
echo "Stop with: sudo systemctl stop aether"
echo "Status with: sudo systemctl status aether"
