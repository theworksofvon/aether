#!/bin/bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "Building Aether containers from $REPO_DIR"
cd "$REPO_DIR"
podman compose -f compose.yaml build
echo "Aether containers built successfully"
