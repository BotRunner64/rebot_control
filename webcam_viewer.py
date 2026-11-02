#!/usr/bin/env python3
"""Display each webcam in its own window."""

import argparse
from pathlib import Path
import struct
import sys
import threading
import time

PREVIEW_WIDTH = 640
PREVIEW_HEIGHT = 480


def positive_int(value):
    number = int(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return number


def discover_cameras():
    """Find Linux video capture nodes, excluding metadata-only nodes."""
    if not sys.platform.startswith("linux"):
        return ["0", "1"]
    import fcntl

    sources = []
    for path in sorted(Path("/dev").glob("video[0-9]*"),
                       key=lambda path: int(path.name[5:])):
        try:
            with path.open("rb", buffering=0) as device:
                data = bytearray(104)
                fcntl.ioctl(device.fileno(), 0x80685600, data)  # VIDIOC_QUERYCAP
            capabilities, device_caps = struct.unpack_from("=II", data, 84)
            if capabilities & 0x80000000:  # V4L2_CAP_DEVICE_CAPS
                capabilities = device_caps
            if capabilities & 0x1001:  # VIDEO_CAPTURE or VIDEO_CAPTURE_MPLANE
                sources.append(str(path))
        except OSError:
            continue
    return sources


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--cameras", nargs="+",
        help="Camera indices or device paths (default: auto-detect on Linux; 0 1 elsewhere).",
    )
    parser.add_argument("--width", type=positive_int, default=1920,
                        help="Requested capture width (default: 1920).")
    parser.add_argument("--height", type=positive_int, default=1080,
                        help="Requested capture height (default: 1080).")
    parser.add_argument("--backend", choices=("auto", "v4l2", "ffmpeg"),
                        default="v4l2" if sys.platform.startswith("linux") else "auto",
                        help="Capture backend (default: v4l2 on Linux; auto elsewhere).")
    parser.add_argument("--fps", type=positive_int, default=30,
                        help="Requested capture frame rate (default: 30).")
    parser.add_argument("--fourcc", choices=("MJPG", "YUYV"),
                        help="Requested pixel format (default: keep device setting).")
    parser.add_argument("--buffer-size", type=positive_int,
                        help="Requested capture buffer size (default: keep backend setting).")
    args = parser.parse_args()
    if args.cameras is None:
        args.cameras = discover_cameras()
    if not args.cameras:
        parser.error("no video capture devices found; specify devices with --cameras")
    if len(set(args.cameras)) != len(args.cameras):
        parser.error("camera sources must be unique")
    return args


def configure_capture(capture, args, cv2, source):
    """Keep backend settings unless a different value is requested."""
    def apply_setting(prop, requested, label):
        if capture.get(prop) == requested:
            return
        accepted = capture.set(prop, requested)
        actual = capture.get(prop)
        if not accepted or actual != requested:
            print(f"Camera {source}: requested {label} {requested}, got {actual:g}.",
                  file=sys.stderr)

    if args.fourcc is not None:
        apply_setting(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*args.fourcc), "format")
    if args.width is not None:
        size = ((cv2.CAP_PROP_FRAME_WIDTH, args.width, "width"),
                (cv2.CAP_PROP_FRAME_HEIGHT, args.height, "height"))
        if any(capture.get(prop) != requested for prop, requested, _ in size):
            # V4L2 commits the size only after both width and height are set.
            accepted = [capture.set(prop, requested) for prop, requested, _ in size]
            for (prop, requested, label), changed in zip(size, accepted):
                actual = capture.get(prop)
                if not changed or actual != requested:
                    print(f"Camera {source}: requested {label} {requested}, got {actual:g}.",
                          file=sys.stderr)
    if args.fps is not None:
        apply_setting(cv2.CAP_PROP_FPS, args.fps, "FPS")
    if args.buffer_size is not None:
        apply_setting(cv2.CAP_PROP_BUFFERSIZE, args.buffer_size, "buffer size")


class CameraReader:
    """Continuously replace one latest-frame slot without blocking other cameras."""

    def __init__(self, source, capture, cv2):
        self.source = source
        self.capture = capture
        self.cv2 = cv2
        self.opened = capture.isOpened()
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.thread = None
        self.frame = None
        self.received_at = 0.0
        self.fps = 0.0

    def start(self):
        thread = threading.Thread(target=self._read, name=f"camera-{self.source}", daemon=True)
        thread.start()
        self.thread = thread

    def _read(self):
        failed = False
        counted = 0
        since = time.monotonic()
        first_frame = True
        try:
            while not self.stop_event.is_set():
                ok, frame = self.capture.read()
                if self.stop_event.is_set():
                    break
                now = time.monotonic()
                available = ok and frame is not None and frame.size > 0
                with self.lock:
                    self.frame = frame if available else None
                    if available:
                        self.received_at = now
                        counted += 1
                    if now - since >= 1:
                        self.fps = counted / (now - since)
                        since, counted = now, 0
                if not available:
                    if not failed:
                        print(f"Camera {self.source}: no frame available.", file=sys.stderr)
                    failed = True
                    self.stop_event.wait(0.05)
                elif failed:
                    print(f"Camera {self.source}: frames resumed.", flush=True)
                    failed = False
                if available and first_frame:
                    print(f"Camera {self.source}: first frame received.", flush=True)
                    first_frame = False
        except self.cv2.error as exc:
            with self.lock:
                self.frame = None
            print(f"Camera {self.source}: capture error: {exc}", file=sys.stderr)
        finally:
            # Only the reader releases its capture; never race release() with read().
            self.capture.release()

    def snapshot(self):
        with self.lock:
            return self.frame, self.received_at, self.fps

    def stop(self):
        self.stop_event.set()

    def close(self):
        self.stop()
        if self.thread is None:
            self.capture.release()
        else:
            self.thread.join(timeout=2)
            if self.thread.is_alive():
                print(f"Camera {self.source}: driver read is still blocked during shutdown.",
                      file=sys.stderr)


class CameraWindow:
    """Present the latest frame using Tk widgets and Pillow, without HighGUI."""

    def __init__(self, root, reader, index, tk, image, image_tk, image_ops):
        self.reader = reader
        self.image = image
        self.image_tk = image_tk
        self.image_ops = image_ops
        self.photo = None
        self.rendered = None
        self.window = tk.Toplevel(root)
        self.window.title(f"Camera {reader.source} - Q / Esc to quit")
        offset = 40 + index * 60
        self.window.geometry(f"{PREVIEW_WIDTH}x{PREVIEW_HEIGHT + 56}+{offset}+{offset}")
        self.window.minsize(160, 120)
        self.window.protocol("WM_DELETE_WINDOW", root.quit)
        self.status = tk.Label(self.window, bg="#202020", fg="white", anchor="w",
                               justify="left", padx=10, pady=8)
        self.status.pack(fill="x")
        self.canvas = tk.Canvas(self.window, bg="black", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)
        self.picture = self.canvas.create_image(0, 0, anchor="center")

    def update(self):
        frame, received_at, fps = self.reader.snapshot()
        age = time.monotonic() - received_at
        available = frame is not None and age < 1
        label = f"Camera {self.reader.source}" + ("" if available else " - no signal")
        status = f"{fps:.1f} FPS"
        if received_at:
            status += f" | Last read {age * 1000:.0f} ms ago"
        self.status.configure(text=f"{label}\n{status}")
        width = max(1, self.canvas.winfo_width())
        height = max(1, self.canvas.winfo_height())
        render_key = (received_at, width, height, available)
        if render_key == self.rendered:
            return
        if available:
            frame_height, frame_width = frame.shape[:2]
            image = self.image.frombytes("RGB", (frame_width, frame_height),
                                         frame.tobytes(), "raw", "BGR")
            image = self.image_ops.contain(image, (width, height),
                                           method=self.image.Resampling.BILINEAR)
            self.photo = self.image_tk.PhotoImage(image, master=self.canvas)
            self.canvas.itemconfigure(self.picture, image=self.photo)
            self.canvas.coords(self.picture, width / 2, height / 2)
        else:
            self.canvas.itemconfigure(self.picture, image="")
            self.photo = None
        self.rendered = render_key


def display(args, cv2, tk, image, image_tk, image_ops):
    cameras = []
    windows = []
    root = None
    refresh_id = None
    try:
        root = tk.Tk()
        root.withdraw()
        for source in args.cameras:
            device = int(source) if source.isdecimal() else source
            if sys.platform.startswith("linux") and isinstance(device, int):
                device = f"/dev/video{device}"
            backend = {"auto": cv2.CAP_ANY, "v4l2": cv2.CAP_V4L2,
                       "ffmpeg": cv2.CAP_FFMPEG}[args.backend]
            capture = cv2.VideoCapture(device, backend)
            reader = CameraReader(source, capture, cv2)
            cameras.append(reader)
            if not reader.opened:
                print(f"Camera {source}: could not open device.", file=sys.stderr)
                continue
            configure_capture(capture, args, cv2, source)
            width = capture.get(cv2.CAP_PROP_FRAME_WIDTH)
            height = capture.get(cv2.CAP_PROP_FRAME_HEIGHT)
            fps = capture.get(cv2.CAP_PROP_FPS)
            fourcc = int(capture.get(cv2.CAP_PROP_FOURCC))
            pixel_format = "".join(chr((fourcc >> (8 * index)) & 0xff) for index in range(4))
            print(f"Camera {source}: backend {capture.getBackendName()}, "
                  f"{width:g}x{height:g}, {fps:g} FPS, "
                  f"format {pixel_format!r}; waiting for first frame.", flush=True)
            reader.start()

        if not any(reader.opened for reader in cameras):
            print("No webcams could be opened. Check device paths and permissions.",
                  file=sys.stderr)
            return 1

        for index, reader in enumerate(cameras):
            windows.append(CameraWindow(root, reader, index, tk, image, image_tk, image_ops))

        def on_key(event):
            if event.keysym.lower() in ("q", "escape"):
                root.quit()

        callback_errors = []

        def on_callback_error(exc_type, exc, traceback):
            callback_errors.append(exc.with_traceback(traceback))
            root.quit()

        def refresh():
            nonlocal refresh_id
            refresh_id = None
            for window in windows:
                window.update()
            # Return to Python regularly so Ctrl-C can interrupt Tk's event loop.
            refresh_id = root.after(16, refresh)

        root.bind_all("<KeyPress>", on_key)
        root.report_callback_exception = on_callback_error
        print("Press Q or Esc, close any camera window, or press Ctrl-C to exit.", flush=True)
        refresh_id = root.after(0, refresh)
        root.mainloop()
        if callback_errors:
            raise callback_errors[0]
        return 0
    finally:
        for reader in cameras:
            reader.stop()
        for reader in cameras:
            reader.close()
        if root is not None:
            if refresh_id is not None:
                root.after_cancel(refresh_id)
            root.destroy()


def main():
    args = parse_args()
    try:
        import tkinter as tk
    except ImportError:
        print("Tkinter is missing. Install python3-tk (system Python) or tk (Conda).",
              file=sys.stderr)
        return 1
    try:
        import cv2
        from PIL import Image, ImageOps, ImageTk
    except ImportError:
        print("Missing dependencies. Run: python -m pip install opencv-python-headless numpy Pillow",
              file=sys.stderr)
        return 1
    try:
        return display(args, cv2, tk, Image, ImageTk, ImageOps)
    except KeyboardInterrupt:
        return 0
    except cv2.error as exc:
        print(f"OpenCV error: {exc}", file=sys.stderr)
        return 1
    except tk.TclError as exc:
        print(f"Tk display error: {exc}. Check Tk installation and desktop access.",
              file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
