#!/usr/bin/env bash
# ============================================================
#  VirtualAGC – macOS setup & launch script (Docker method)
#  Run this from your terminal:  bash run-virtualagc.sh
# ============================================================
set -euo pipefail

REPO_URL="https://github.com/virtualagc/virtualagc"
BRANCH="master"
WORK_DIR="$HOME/virtualagc"

echo ""
echo "🚀  VirtualAGC setup script"
echo "============================================================"

# ── 1. Check dependencies ────────────────────────────────────
echo ""
echo "▶ Checking dependencies..."

if ! command -v git &> /dev/null; then
  echo "  ✗  git is not installed. Install Xcode Command Line Tools: xcode-select --install"
  exit 1
fi
echo "  ✓  git found."

if ! docker info > /dev/null 2>&1; then
  echo "  ✗  Docker is not running. Please start Docker Desktop and try again."
  exit 1
fi
echo "  ✓  Docker is running."

# Support both Docker Compose v1 (docker-compose) and v2 (docker compose)
if command -v docker-compose &> /dev/null; then
  COMPOSE="docker-compose"
elif docker compose version &> /dev/null 2>&1; then
  COMPOSE="docker compose"
else
  echo "  ✗  Docker Compose not found. Please update Docker Desktop."
  exit 1
fi
echo "  ✓  $COMPOSE found."

# ── 2. Checkout the source code ──────────────────────────────
echo ""
echo "▶ Checking out VirtualAGC source ($BRANCH)..."

if [ -d "$WORK_DIR/.git" ]; then
  echo "  Repo exists — fetching latest changes..."
  git -C "$WORK_DIR" fetch origin "$BRANCH"
  git -C "$WORK_DIR" checkout "$BRANCH"
  git -C "$WORK_DIR" reset --hard "origin/$BRANCH"
  echo "  ✓  Up to date."
else
  [ -d "$WORK_DIR" ] && rm -rf "$WORK_DIR"
  git clone --branch "$BRANCH" --depth 1 "$REPO_URL" "$WORK_DIR"
  echo "  ✓  Checked out to $WORK_DIR"
fi

# ── 3. Build & start the container ───────────────────────────
echo ""
echo "▶ Building and starting VirtualAGC container (first build takes ~5 minutes)..."
cd "$WORK_DIR/Docker"
$COMPOSE up -d --build

# ── 4. Wait for the service to be ready ──────────────────────
echo ""
echo "▶ Waiting for noVNC to become available..."
for i in $(seq 1 30); do
  if curl -s --max-time 2 http://localhost:6080 > /dev/null 2>&1; then
    break
  fi
  sleep 2
done

# ── 5. Open in browser ────────────────────────────────────────
echo ""
echo "============================================================"
echo "  ✅  VirtualAGC is running!"
echo ""
echo "  Open this URL in your browser:"
echo "  👉  http://localhost:6080/vnc.html"
echo ""
echo "  Stop:    $COMPOSE -f $WORK_DIR/Docker/docker-compose.yml down"
echo "  Restart: $COMPOSE -f $WORK_DIR/Docker/docker-compose.yml up -d"
echo "============================================================"
echo ""

# Open the browser automatically on macOS
open "http://localhost:6080/vnc.html" 2>/dev/null || true
