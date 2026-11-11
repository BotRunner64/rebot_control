"""Prepare reusable, lightweight videos outside the raw dataset."""

from concurrent.futures import ThreadPoolExecutor
import fcntl
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
import threading


VIDEO_SCRIPT = Path(__file__).with_name("rebot_process_video.py")
PREVIEW_VERSION = "640px-15fps-baseline-v1"


class PreviewCache:
    def __init__(self, directory):
        self.directory = Path(directory)
        self.pool = ThreadPoolExecutor(max_workers=2, thread_name_prefix="preview")
        self.guard = threading.Lock()
        self.closed = threading.Event()
        self.tasks = {}
        self.processes = set()

    def path(self, source):
        source = Path(source)
        stat = source.stat()
        identity = json.dumps([PREVIEW_VERSION, str(source.resolve()), stat.st_size, stat.st_mtime_ns])
        return self.directory / (hashlib.sha256(identity.encode()).hexdigest() + ".mp4")

    def prepare(self, sources):
        with self.guard:
            if self.closed.is_set():
                return {"state": "error", "error": "Preview preparation cancelled."}
            pending = False
            for source in sources:
                target = self.path(source)
                if target.is_file() and target.stat().st_size > 0:
                    continue
                task = self.tasks.get(target)
                if task is None or task.done() and task.exception() is None:
                    task = self.tasks[target] = self.pool.submit(self.build, source, target)
                if task.done():
                    error = task.exception()
                    if error:
                        return {"state": "error", "error": str(error)}
                else:
                    pending = True
            return {"state": "preparing" if pending else "ready"}

    def build(self, source, target):
        self.directory.mkdir(parents=True, exist_ok=True)
        with target.with_suffix(".lock").open("a") as lock:
            while True:
                if self.closed.is_set():
                    raise RuntimeError("Preview preparation cancelled.")
                try:
                    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    break
                except BlockingIOError:
                    self.closed.wait(.1)
            if target.is_file() and target.stat().st_size > 0:
                return
            handle, temporary = tempfile.mkstemp(prefix=".preview-", suffix=".mp4", dir=self.directory)
            os.close(handle)
            process = None
            try:
                with self.guard:
                    if self.closed.is_set():
                        raise RuntimeError("Preview preparation cancelled.")
                    process = subprocess.Popen(
                        ["/usr/bin/python3", "-B", str(VIDEO_SCRIPT)], stdin=subprocess.PIPE,
                        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                    self.processes.add(process)
                request = {"preview": True, "source": str(source), "output": temporary}
                _, error = process.communicate(json.dumps(request), timeout=1810)
                if process.returncode:
                    raise RuntimeError(error.strip() or "Preview conversion failed.")
                if self.closed.is_set():
                    raise RuntimeError("Preview preparation cancelled.")
                os.replace(temporary, target)
            finally:
                if process is not None:
                    if process.poll() is None:
                        process.kill(); process.communicate()
                    with self.guard:
                        self.processes.discard(process)
                Path(temporary).unlink(missing_ok=True)

    def close(self):
        with self.guard:
            self.closed.set()
            for process in self.processes:
                if process.poll() is None:
                    process.terminate()
        self.pool.shutdown(wait=True, cancel_futures=True)
