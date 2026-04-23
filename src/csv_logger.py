"""
CSV File Logger — Daily-rotating CSV logging for AppDaemon apps.

Writes one row per tick to a date-stamped CSV file.  A new file
(with a fresh header row) is created at midnight UTC.  Each app
gets its own logger instance with its own column schema.

Usage::

    logger = CsvLogger("/config/logs", "controller", ["grid_w", "soc_pct"])
    logger.log_row({"grid_w": 123, "soc_pct": 55.0})

Files are named ``<prefix>_YYYY-MM-DD.csv``.
"""

from __future__ import annotations

import csv
import io
from datetime import datetime, timezone
from pathlib import Path


class CsvLogger:
    """Daily-rotating CSV file logger.

    Each instance manages one log stream identified by *prefix*
    (e.g. ``"controller"`` or ``"driver"``).  Rows are flushed
    immediately so data survives crashes.

    Attributes:
        log_dir:  Directory for CSV files.
        prefix:   Filename prefix (e.g. ``"zfi_controller"``).
        columns:  Ordered list of field names (excluding timestamp).
    """

    def __init__(
        self,
        log_dir: str,
        prefix: str,
        columns: list[str],
    ) -> None:
        self.log_dir = Path(log_dir)
        self.prefix = prefix
        self.columns = columns
        self._current_date: str = ""
        self._file: io.TextIOWrapper | None = None
        self._writer: csv.writer | None = None

    # ─── Public API ──────────────────────────────────────

    def log_row(self, values: dict[str, object]) -> None:
        """Append one row.  *values* is a dict mapping column names to values.

        Missing columns are written as empty strings.  A ``timestamp``
        column is prepended automatically (ISO 8601, UTC).
        """
        now = datetime.now(timezone.utc)
        today = now.strftime("%Y-%m-%d")

        if today != self._current_date:
            self._rotate(today)

        assert self._writer is not None
        row = [now.strftime("%Y-%m-%dT%H:%M:%S")]
        for col in self.columns:
            row.append(values.get(col, ""))
        self._writer.writerow(row)
        assert self._file is not None
        self._file.flush()

    def close(self) -> None:
        """Flush and close the current file."""
        if self._file is not None:
            self._file.close()
            self._file = None
            self._writer = None
            self._current_date = ""

    # ─── Internal ────────────────────────────────────────

    def _rotate(self, today: str) -> None:
        """Close the old file (if any) and open a new one for *today*."""
        self.close()
        self.log_dir.mkdir(parents=True, exist_ok=True)
        path = self.log_dir / f"{self.prefix}_{today}.csv"
        file_exists = path.exists() and path.stat().st_size > 0
        self._file = open(path, "a", newline="", encoding="utf-8")  # noqa: SIM115
        self._writer = csv.writer(self._file)
        if not file_exists:
            self._writer.writerow(["timestamp"] + self.columns)
            self._file.flush()
        self._current_date = today
