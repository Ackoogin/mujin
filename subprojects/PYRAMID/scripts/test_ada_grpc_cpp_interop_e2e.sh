#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SERVER_BIN=""
CLIENT_BIN=""
TRANSPORT_PLUGIN=""
CODEC_PLUGIN=""
ADDRESS="auto"
TIMEOUT_SEC=20

while [[ $# -gt 0 ]]; do
  case "$1" in
    --server-bin) SERVER_BIN="$2"; shift 2 ;;
    --client-bin) CLIENT_BIN="$2"; shift 2 ;;
    --transport-plugin) TRANSPORT_PLUGIN="$2"; shift 2 ;;
    --codec-plugin) CODEC_PLUGIN="$2"; shift 2 ;;
    --address) ADDRESS="$2"; shift 2 ;;
    --timeout) TIMEOUT_SEC="$2"; shift 2 ;;
    *) shift ;;
  esac
done

if [[ -z "$SERVER_BIN" || ! -x "$SERVER_BIN" ]]; then
  echo "[ada-grpc-driver] FAIL: server binary not executable: $SERVER_BIN" >&2
  exit 1
fi
if [[ -z "$CLIENT_BIN" || ! -x "$CLIENT_BIN" ]]; then
  echo "[ada-grpc-driver] FAIL: Ada client binary not executable: $CLIENT_BIN" >&2
  exit 1
fi
if [[ -z "$TRANSPORT_PLUGIN" || ! -f "$TRANSPORT_PLUGIN" ]]; then
  echo "[ada-grpc-driver] FAIL: transport plugin not found: $TRANSPORT_PLUGIN" >&2
  exit 1
fi
if [[ -z "$CODEC_PLUGIN" || ! -f "$CODEC_PLUGIN" ]]; then
  echo "[ada-grpc-driver] FAIL: codec plugin not found: $CODEC_PLUGIN" >&2
  exit 1
fi

if [[ "$ADDRESS" == "auto" || -z "$ADDRESS" ]]; then
  ADDRESS="127.0.0.1:$(python3 "$SCRIPT_DIR/pick_loopback_port.py")"
fi

READY_FILE="$(mktemp "${TMPDIR:-/tmp}/pyramid_grpc_ready.XXXXXX")"
rm -f "$READY_FILE"
SERVER_PID=""
cleanup() {
  if [[ -n "$SERVER_PID" ]] && kill -0 "$SERVER_PID" 2>/dev/null; then
    kill "$SERVER_PID" 2>/dev/null || true
    wait "$SERVER_PID" 2>/dev/null || true
  fi
  rm -f "$READY_FILE"
}
trap cleanup EXIT

echo "[ada-grpc-driver] starting gRPC server on $ADDRESS"
"$SERVER_BIN" --address "$ADDRESS" --ready-file "$READY_FILE" --timeout "$TIMEOUT_SEC" &
SERVER_PID=$!

deadline=$((SECONDS + 10))
while [[ $SECONDS -lt $deadline ]]; do
  if [[ -s "$READY_FILE" ]]; then
    break
  fi
  if ! kill -0 "$SERVER_PID" 2>/dev/null; then
    echo "[ada-grpc-driver] FAIL: server exited before ready" >&2
    exit 1
  fi
  sleep 0.1
done

if [[ ! -s "$READY_FILE" ]]; then
  echo "[ada-grpc-driver] FAIL: server did not become ready" >&2
  exit 1
fi

echo "[ada-grpc-driver] running Ada client"
"$CLIENT_BIN" \
  --transport-plugin "$TRANSPORT_PLUGIN" \
  --codec-plugin "$CODEC_PLUGIN" \
  --address "$ADDRESS"

echo "[ada-grpc-driver] PASS"
