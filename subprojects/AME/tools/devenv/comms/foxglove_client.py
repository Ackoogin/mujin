"""WebSocket client for the AME FoxgloveBridge.

Connects to ws://localhost:8765 (configurable), subscribes to /bt_events
and /wm_audit channels, and queues decoded JSON events for UI consumption.

The Foxglove WebSocket protocol is documented at:
https://github.com/foxglove/ws-protocol

Message flow:
  1. Server sends ServerInfo on connect
  2. Server sends Advertise with available channels
  3. Client sends Subscribe for desired channel IDs
  4. Server streams MessageData frames (binary: 1-byte opcode + payload)
"""

from __future__ import annotations

import json
import struct
import threading
import time
from collections import deque
from typing import Callable, Optional

import websocket


# Foxglove WS protocol opcodes (binary messages)
_OP_MESSAGE_DATA = 1

# Foxglove WS protocol opcodes (text/JSON messages from server)
_SERVER_INFO = "serverInfo"
_ADVERTISE = "advertise"
_UNADVERTISE = "unadvertise"

# Foxglove WS protocol opcodes (text/JSON messages to server)
_SUBSCRIBE = "subscribe"


class FoxgloveClient:
    """Async Foxglove WebSocket client with thread-safe event queues.

    Usage::

        client = FoxgloveClient("ws://localhost:8765")
        client.start()

        # In UI loop:
        for evt in client.drain_bt_events():
            ...  # dict with ts_us, node, type, prev, status, etc.
        for evt in client.drain_wm_events():
            ...  # dict with wm_version, ts_us, fact, value, source

        client.stop()
    """

    def __init__(
        self,
        url: str = "ws://localhost:8765",
        buffer_size: int = 10_000,
        on_connected: Optional[Callable[[], None]] = None,
        on_disconnected: Optional[Callable[[], None]] = None,
    ):
        self._url = url
        self._buffer_size = buffer_size
        self._on_connected = on_connected
        self._on_disconnected = on_disconnected

        self._bt_queue: deque[dict] = deque(maxlen=buffer_size)
        self._wm_queue: deque[dict] = deque(maxlen=buffer_size)

        self._ws: Optional[websocket.WebSocketApp] = None
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._connected = False

        # channel_id -> channel_topic mapping (learned from Advertise)
        self._channels: dict[int, str] = {}
        # subscription_id -> channel_id
        self._subscriptions: dict[int, int] = {}
        self._next_sub_id = 1

    @property
    def connected(self) -> bool:
        return self._connected

    def start(self) -> None:
        """Connect in a background thread."""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(
            target=self._run, name="foxglove-ws", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        """Disconnect and join the background thread."""
        self._running = False
        if self._ws:
            self._ws.close()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=3.0)
        self._connected = False

    def drain_bt_events(self) -> list[dict]:
        """Non-blocking drain of all queued BT events."""
        events = []
        while self._bt_queue:
            try:
                events.append(self._bt_queue.popleft())
            except IndexError:
                break
        return events

    def drain_wm_events(self) -> list[dict]:
        """Non-blocking drain of all queued WM audit events."""
        events = []
        while self._wm_queue:
            try:
                events.append(self._wm_queue.popleft())
            except IndexError:
                break
        return events

    # -- internal ------------------------------------------------------------

    def _run(self) -> None:
        """Background thread: connect with auto-reconnect."""
        while self._running:
            try:
                self._ws = websocket.WebSocketApp(
                    self._url,
                    on_open=self._on_open,
                    on_message=self._on_message,
                    on_error=self._on_error,
                    on_close=self._on_close,
                )
                self._ws.run_forever(ping_interval=10, ping_timeout=5)
            except Exception:
                pass
            # Reconnect backoff
            if self._running:
                time.sleep(2.0)

    def _on_open(self, ws: websocket.WebSocketApp) -> None:
        self._connected = True
        if self._on_connected:
            self._on_connected()

    def _on_close(self, ws: websocket.WebSocketApp, code, msg) -> None:
        self._connected = False
        self._channels.clear()
        self._subscriptions.clear()
        if self._on_disconnected:
            self._on_disconnected()

    def _on_error(self, ws: websocket.WebSocketApp, error) -> None:
        pass  # reconnect handled in _run loop

    def _on_message(self, ws: websocket.WebSocketApp, message) -> None:
        if isinstance(message, str):
            self._handle_text_message(message)
        elif isinstance(message, bytes):
            self._handle_binary_message(message)

    def _handle_text_message(self, raw: str) -> None:
        try:
            msg = json.loads(raw)
        except json.JSONDecodeError:
            return

        op = msg.get("op", "")

        if op == _ADVERTISE:
            for ch in msg.get("channels", []):
                ch_id = ch.get("id")
                topic = ch.get("topic", "")
                if ch_id is not None:
                    self._channels[ch_id] = topic
            # Auto-subscribe to known channels
            self._subscribe_to_known_channels()

        elif op == _SERVER_INFO:
            pass  # informational only

    def _subscribe_to_known_channels(self) -> None:
        """Subscribe to /bt_events and /wm_audit if advertised."""
        subs = []
        for ch_id, topic in self._channels.items():
            if topic in ("/bt_events", "/wm_audit"):
                already = any(
                    cid == ch_id for cid in self._subscriptions.values()
                )
                if not already:
                    sub_id = self._next_sub_id
                    self._next_sub_id += 1
                    self._subscriptions[sub_id] = ch_id
                    subs.append({"id": sub_id, "channelId": ch_id})

        if subs and self._ws:
            self._ws.send(json.dumps({"op": _SUBSCRIBE, "subscriptions": subs}))

    def _handle_binary_message(self, data: bytes) -> None:
        if len(data) < 1:
            return
        opcode = data[0]
        if opcode != _OP_MESSAGE_DATA:
            return

        # Binary MessageData layout after opcode byte:
        # uint32 subscriptionId, uint64 timestamp, bytes payload
        if len(data) < 13:
            return
        sub_id = struct.unpack_from("<I", data, 1)[0]
        # skip timestamp (8 bytes at offset 5)
        payload = data[13:]

        ch_id = self._subscriptions.get(sub_id)
        if ch_id is None:
            return
        topic = self._channels.get(ch_id, "")

        try:
            evt = json.loads(payload)
        except (json.JSONDecodeError, UnicodeDecodeError):
            return

        if topic == "/bt_events":
            self._bt_queue.append(evt)
        elif topic == "/wm_audit":
            self._wm_queue.append(evt)
