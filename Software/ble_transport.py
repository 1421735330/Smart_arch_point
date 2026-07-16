"""Small Bleak adapter running all Bluetooth work on one asyncio thread."""

from __future__ import annotations

import asyncio
from collections.abc import Callable
import threading
from typing import Any

try:
    from bleak import BleakClient, BleakScanner
except ImportError:  # The GUI demo remains usable before dependencies are installed.
    BleakClient = None  # type: ignore[assignment]
    BleakScanner = None  # type: ignore[assignment]


NOTIFY_CHARACTERISTIC_UUID = "0000ffe2-0000-1000-8000-00805f9b34fb"


class BleTransport:
    """Expose non-blocking BLE operations to a Tkinter application."""

    def __init__(self, event_callback: Callable[[str, Any], None]) -> None:
        self._emit = event_callback
        self._devices: dict[str, Any] = {}
        self._client: Any = None
        self._closing = False
        self._ready = threading.Event()
        self._loop: asyncio.AbstractEventLoop | None = None
        self._thread = threading.Thread(target=self._thread_main, daemon=True)
        self._thread.start()
        self._ready.wait(timeout=2.0)

    @property
    def dependency_available(self) -> bool:
        return BleakClient is not None and BleakScanner is not None

    def scan(self, timeout: float = 5.0) -> None:
        self._submit(self._scan(timeout), "扫描")

    def connect(self, address: str) -> None:
        self._submit(self._connect(address), "连接")

    def disconnect(self) -> None:
        self._submit(self._disconnect(), "断开")

    def close(self) -> None:
        if self._closing:
            return
        self._closing = True
        if self._loop is not None and self._loop.is_running():
            future = asyncio.run_coroutine_threadsafe(self._disconnect(), self._loop)
            try:
                future.result(timeout=2.0)
            except Exception:
                pass
            self._loop.call_soon_threadsafe(self._loop.stop)
        self._thread.join(timeout=2.0)

    def _thread_main(self) -> None:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        self._loop = loop
        self._ready.set()
        try:
            loop.run_forever()
        finally:
            pending = asyncio.all_tasks(loop)
            for task in pending:
                task.cancel()
            if pending:
                loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
            loop.close()

    def _submit(self, coroutine: Any, operation: str) -> None:
        if self._closing or self._loop is None:
            coroutine.close()
            return
        future = asyncio.run_coroutine_threadsafe(coroutine, self._loop)

        def report_error(done: Any) -> None:
            try:
                done.result()
            except Exception as exc:
                self._emit("error", f"{operation}失败：{exc}")

        future.add_done_callback(report_error)

    async def _scan(self, timeout: float) -> None:
        if BleakScanner is None:
            raise RuntimeError("缺少 Bleak，请先运行 setup.ps1")
        self._emit("status", "正在扫描 BLE 设备…")
        discovered = await BleakScanner.discover(timeout=timeout, return_adv=True)
        rows: list[dict[str, Any]] = []
        self._devices.clear()
        for device, advertisement in discovered.values():
            self._devices[device.address] = device
            rows.append(
                {
                    "address": device.address,
                    "name": device.name or advertisement.local_name or "未命名设备",
                    "rssi": advertisement.rssi,
                }
            )
        rows.sort(key=lambda item: item["rssi"], reverse=True)
        self._emit("devices", rows)
        self._emit("status", f"扫描完成：发现 {len(rows)} 个设备")

    async def _connect(self, address: str) -> None:
        if BleakClient is None:
            raise RuntimeError("缺少 Bleak，请先运行 setup.ps1")
        device = self._devices.get(address)
        if device is None:
            raise RuntimeError("设备信息已失效，请重新扫描")
        await self._disconnect()
        self._emit("status", f"正在连接 {device.name or address}…")
        client = BleakClient(device, disconnected_callback=self._on_disconnected)
        await client.connect()
        try:
            await client.start_notify(NOTIFY_CHARACTERISTIC_UUID, self._on_notification)
        except Exception:
            await client.disconnect()
            raise
        self._client = client
        self._emit("connected", {"address": address, "name": device.name or address})

    async def _disconnect(self) -> None:
        client, self._client = self._client, None
        if client is not None and client.is_connected:
            try:
                await client.stop_notify(NOTIFY_CHARACTERISTIC_UUID)
            except Exception:
                pass
            await client.disconnect()
        self._emit("disconnected", None)

    def _on_notification(self, _sender: Any, data: bytearray) -> None:
        self._emit("data", bytes(data))

    def _on_disconnected(self, client: Any) -> None:
        if client is self._client:
            self._client = None
            self._emit("disconnected", None)
