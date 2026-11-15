"""Show both Star Arm leaders in the 3D viewer, or drive virtual reBot RS arms with them.

Reached through `rebot leader view`; no follower hardware is involved.
"""

from collections import deque
from contextlib import ExitStack
import math
import struct
import sys
import time

from rebot.bindings import SIDES, select_leaders
from rebot.devices.stararm import angle_difference, open_leader, read_angles, release_torque
from rebot.paths import LEADER_BINDINGS
from rebot.viewer.dual import SCENES, load_dual_model, pose_from_leaders
from rebot.viewer.scene import SceneViewer
from rebot.viewer.server import validate_port


class HoverState:
    """Hold both arms when still and closed; opening either handle releases both."""

    def __init__(self, references):
        self.references = references
        self.samples = deque(maxlen=25)
        self.previous = None
        self.still_since = None
        self.holding = False
        self.closed = {side: False for side in SIDES}

    def reset_stillness(self):
        self.samples.clear()
        self.previous = None
        self.still_since = None

    def update(self, buses, raw):
        # The calibrated default pose defines each handle's closed position.
        gaps = {side: abs(angle_difference([raw[side][6]], [self.references[side][6]])[0])
                for side in SIDES}
        for side, gap in gaps.items():
            if gap <= 5:
                self.closed[side] = True
            elif gap >= 10:
                self.closed[side] = False

        if self.holding:
            if any(gap >= 10 for gap in gaps.values()):
                for side in SIDES:
                    release_torque(buses[side])
                self.holding = False
                self.reset_stillness()
                print("Handle opened; both hover unlock commands sent.", flush=True)
            return

        # Per-arm stillness thresholds, with one shared timer.
        self.samples.append([angle for side in SIDES for angle in raw[side][:6]])
        filtered = [sum(values) / len(self.samples) for values in zip(*self.samples)]
        changes = ([sum(abs(x) for x in angle_difference(filtered[i:i+6], self.previous[i:i+6]))
                    for i in (0, 6)] if self.previous is not None else [math.inf, math.inf])
        self.previous = filtered
        now = time.monotonic()
        if any(change > .1 for change in changes):
            self.still_since = None
        elif self.still_since is None:
            self.still_since = now
        elif now - self.still_since > 1 and all(self.closed.values()):
            for side in SIDES:
                # Hold raw hardware angles, not the calibrated model offsets.
                commands = [struct.pack("<BiIHHH", i, int(raw[side][i] * 10), 100, 50, 50, 4095)
                            for i in range(6)]
                buses[side].send_sync_multiturnanglebyinterval(14, 6, commands)
            self.holding = True
            print("Both arms still and handles closed; both hover hold commands sent.", flush=True)


def release_on_exit(side, bus):
    try:
        release_torque(bus)
    except Exception as exc:
        print(f"{side}: exit unlock failed: {exc}", file=sys.stderr, flush=True)
    else:
        print(f"{side}: exit unlock command sent.", flush=True)


def view(mode="observe", config=LEADER_BINDINGS, target="stararm", *, port=8080, open_browser=True):
    validate_port(port)
    if mode not in ("observe", "hover"):
        raise ValueError(f"Unknown view mode: {mode}")
    if target not in SCENES:
        raise ValueError(f"Unknown view target: {target}")
    leaders = select_leaders(config, "both")
    references = {side: leaders[side].reference_deg for side in SIDES}

    def relative(raw):
        # Preserve the SDK's multi-turn angles: wrapping before RS limits can
        # jump from the upper limit to the lower limit at +/-180 degrees.
        return {side: [angle - reference for angle, reference in zip(raw[side], references[side])]
                for side in SIDES}

    model, data = load_dual_model(target)
    with ExitStack() as stack:
        buses = {}
        for side in SIDES:
            buses[side] = stack.enter_context(open_leader(leaders[side].port, leaders[side].baudrate))
            if mode == "hover":
                # Registered after the serial context: unlock before closing it.
                stack.callback(release_on_exit, side, buses[side])
        raw = {side: read_angles(buses[side]) for side in SIDES}
        print("Releasing both arms before opening the viewer. Support the physical arms.", flush=True)
        for side, bus in buses.items():
            release_torque(bus)
            print(f"{side}: startup unlock command sent.", flush=True)
        pose_from_leaders(model, data, relative(raw), target)
        with SceneViewer(model, data, target, port=port, open_browser=open_browser) as viewer:
            hovering = HoverState(references) if mode == "hover" else None
            if hovering is not None:
                print("Hover enabled: both arms still and both handles closed -> hold both arms. "
                      "Open either handle -> release both arms. Exit unlocks both arms.", flush=True)
            label = "reBot RS" if target == "rebot" else "Star Arm"
            print(f"Star Arm -> 3D viewer {label} dual-arm display started. "
                  "Move both leaders; use Close viewer or Ctrl-C to stop.", flush=True)
            started, frames, failures = time.monotonic(), 0, 0
            last_valid = started
            while viewer.is_running():
                tick = time.monotonic()
                try:
                    raw = {}
                    for side in SIDES:
                        raw[side] = read_angles(buses[side])
                except ValueError as exc:
                    if hovering is not None:
                        hovering.reset_stillness()
                    failures += 1
                    elapsed = time.monotonic() - last_valid
                    detail = f"{side}: {exc}"
                    print(f"Feedback retry {failures}: {detail} "
                          f"({elapsed:.2f}s since last complete frame); models frozen.",
                          file=sys.stderr, flush=True)
                    # One SDK read can itself block for ~0.8 seconds. Require
                    # repeated failed reads as well as sustained loss of data.
                    if failures >= 3 and elapsed >= 2:
                        raise ValueError(f"Feedback did not recover after {failures} reads "
                                         f"and {elapsed:.2f}s; stopping. Last error: {detail}") from exc
                else:
                    last_valid = time.monotonic()
                    if failures:
                        print(f"Feedback recovered after {failures} failed read(s); resuming.", flush=True)
                    failures = 0
                    if hovering is not None:
                        hovering.update(buses, raw)
                    pose_from_leaders(model, data, relative(raw), target)
                    frames += 1
                viewer.sync()
                time.sleep(max(0, 1 / 30 - (time.monotonic() - tick)))
            print(f"Received {frames} paired frames in {time.monotonic() - started:.1f} seconds.")
