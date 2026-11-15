"""Native SocketCAN interface discovery and configuration."""

import json
import os
from pathlib import Path
import subprocess

NET = Path("/sys/class/net")
BITRATE = 1_000_000


def channels():
    return sorted(p.name for p in NET.iterdir()
                  if (p / "type").read_text().strip() == "280")


def device_path(channel):
    """Stable USB device path of an interface; interface names change across replugs."""
    path = NET / channel / "device"
    return str(path.resolve()) if path.exists() else ""


def check_channel(channel, configure=False):
    """Require an interface that is UP at 1 Mbps; optionally configure it with sudo."""
    if channel not in channels():
        raise ValueError(f"No native CAN interface named {channel}.")
    result = subprocess.run(["ip", "-details", "-json", "link", "show", "dev", channel],
                            check=True, capture_output=True, text=True)
    entry, = json.loads(result.stdout)
    info = entry.get("linkinfo", {})
    data = info.get("info_data", {})
    if ("UP" in entry.get("flags", []) and info.get("info_kind") == "can"
            and data.get("bittiming", {}).get("bitrate") == BITRATE
            and data.get("state") not in ("BUS-OFF", "STOPPED")):
        return
    if not configure:
        raise ValueError(
            f"{channel} must be UP at {BITRATE} bit/s. Run `rebot arm scan`, or configure it:\n"
            f"  sudo ip link set {channel} down\n"
            f"  sudo ip link set {channel} type can bitrate {BITRATE}\n"
            f"  sudo ip link set {channel} up")
    print(f"Configuring {channel} at {BITRATE} bit/s...", flush=True)
    prefix = [] if os.geteuid() == 0 else ["sudo"]
    try:
        for options in (["down"], ["type", "can", "bitrate", str(BITRATE)], ["up"]):
            subprocess.run(prefix + ["ip", "link", "set", channel] + options, check=True)
    except subprocess.CalledProcessError as exc:
        raise RuntimeError(
            f"Could not configure {channel} (exit {exc.returncode}). "
            "Run scan in a terminal with sudo access, then retry.") from exc
    # Verify the kernel accepted the configuration before opening CAN.
    check_channel(channel)


def link_report(channel):
    """One line of kernel link state and counters, for diagnosing a silent bus."""
    result = subprocess.run(
        ["ip", "-details", "-statistics", "-json", "link", "show", "dev", channel],
        check=True, capture_output=True, text=True)
    entry, = json.loads(result.stdout)
    data = entry.get("linkinfo", {}).get("info_data", {})
    stats = entry.get("stats64", entry.get("stats", {}))
    errors = data.get("berr_counter", {})
    return (f"CAN state: {data.get('state', 'unknown')}; "
            f"total RX packets: {stats.get('rx', {}).get('packets', 'unknown')}; "
            f"total TX packets: {stats.get('tx', {}).get('packets', 'unknown')}; "
            f"error counters: tx={errors.get('tx', 'unknown')} rx={errors.get('rx', 'unknown')}")
