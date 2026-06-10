"""Thread-safe JSONL logger for per-run grasp session recording."""
from __future__ import annotations

import json
import os
import threading
from datetime import datetime, timezone
from pathlib import Path


def _to_serializable(obj):
    try:
        import numpy as np
        if isinstance(obj, np.integer):
            return int(obj)
        if isinstance(obj, np.floating):
            return float(obj)
        if isinstance(obj, np.ndarray):
            return obj.tolist()
    except ImportError:
        pass
    return str(obj)


class GraspLogger:
    """Writes one JSON line per event to a timestamped file under log_dir."""

    def __init__(self, log_dir: str) -> None:
        expanded = os.path.expanduser(log_dir)
        Path(expanded).mkdir(parents=True, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._path = os.path.join(expanded, f"grasp_{ts}.jsonl")
        self._file = open(self._path, "w", buffering=1)  # line-buffered
        self._lock = threading.Lock()

    @property
    def path(self) -> str:
        return self._path

    def write(self, event: str, **fields) -> None:
        entry = {"ts": datetime.now(timezone.utc).isoformat(), "event": event, **fields}
        line = json.dumps(entry, default=_to_serializable) + "\n"
        with self._lock:
            self._file.write(line)

    def close(self) -> None:
        with self._lock:
            self._file.flush()
            self._file.close()
