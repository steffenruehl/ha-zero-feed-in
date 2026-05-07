"""Tests for csv_logger — CsvLogger with daily rotation."""

from __future__ import annotations

import csv
import gzip
from pathlib import Path
from unittest.mock import patch

import pytest

from src.csv_logger import CsvLogger


@pytest.fixture
def log_dir(tmp_path: Path) -> Path:
    """Return a temporary directory for log files."""
    d = tmp_path / "logs"
    d.mkdir()
    return d


class TestCsvLogger:
    def test_creates_file_with_header(self, log_dir: Path):
        logger = CsvLogger(str(log_dir), "test", ["a", "b"])
        logger.log_row({"a": 1, "b": 2})
        logger.close()

        files = list(log_dir.glob("test_*.csv"))
        assert len(files) == 1

        rows = list(csv.reader(files[0].open()))
        assert rows[0] == ["timestamp", "a", "b"]
        assert rows[1][1:] == ["1", "2"]

    def test_appends_rows(self, log_dir: Path):
        logger = CsvLogger(str(log_dir), "test", ["x"])
        logger.log_row({"x": 10})
        logger.log_row({"x": 20})
        logger.log_row({"x": 30})
        logger.close()

        files = list(log_dir.glob("test_*.csv"))
        rows = list(csv.reader(files[0].open()))
        # 1 header + 3 data rows
        assert len(rows) == 4

    def test_missing_column_written_as_empty(self, log_dir: Path):
        logger = CsvLogger(str(log_dir), "test", ["a", "b", "c"])
        logger.log_row({"a": 1})  # b and c missing
        logger.close()

        files = list(log_dir.glob("test_*.csv"))
        rows = list(csv.reader(files[0].open()))
        assert rows[1][1:] == ["1", "", ""]

    def test_creates_directory_if_missing(self, tmp_path: Path):
        deep = tmp_path / "a" / "b" / "c"
        logger = CsvLogger(str(deep), "test", ["v"])
        logger.log_row({"v": 42})
        logger.close()
        assert deep.exists()
        assert len(list(deep.glob("test_*.csv"))) == 1

    def test_close_is_idempotent(self, log_dir: Path):
        logger = CsvLogger(str(log_dir), "test", ["x"])
        logger.log_row({"x": 1})
        logger.close()
        logger.close()  # should not raise

    def test_reopen_appends_without_duplicate_header(self, log_dir: Path):
        """Closing and reopening the same day's file doesn't re-write the header."""
        logger1 = CsvLogger(str(log_dir), "test", ["x"])
        logger1.log_row({"x": 1})
        logger1.close()

        # Second logger targeting same dir/prefix/day
        logger2 = CsvLogger(str(log_dir), "test", ["x"])
        logger2.log_row({"x": 2})
        logger2.close()

        files = list(log_dir.glob("test_*.csv"))
        assert len(files) == 1
        rows = list(csv.reader(files[0].open()))
        # 1 header + 2 data rows (not 2 headers)
        assert len(rows) == 3
        assert rows[0] == ["timestamp", "x"]

    def test_separate_prefixes_separate_files(self, log_dir: Path):
        ctrl = CsvLogger(str(log_dir), "zfi_controller", ["grid_w"])
        drv = CsvLogger(str(log_dir), "zfi_driver", ["desired_w"])
        ctrl.log_row({"grid_w": 100})
        drv.log_row({"desired_w": 200})
        ctrl.close()
        drv.close()

        ctrl_files = list(log_dir.glob("zfi_controller_*.csv"))
        drv_files = list(log_dir.glob("zfi_driver_*.csv"))
        assert len(ctrl_files) == 1
        assert len(drv_files) == 1

    def test_timestamp_is_iso_format(self, log_dir: Path):
        logger = CsvLogger(str(log_dir), "test", ["x"])
        logger.log_row({"x": 1})
        logger.close()

        files = list(log_dir.glob("test_*.csv"))
        rows = list(csv.reader(files[0].open()))
        ts = rows[1][0]
        # Should look like 2026-04-19T12:34:56
        assert "T" in ts
        assert len(ts) == 19

    def test_rotate_compresses_previous_day(self, log_dir: Path):
        """When the date changes, the previous day's CSV is gzip-compressed."""
        logger = CsvLogger(str(log_dir), "test", ["x"])

        # Day 1
        with patch("src.csv_logger.datetime") as mock_dt:
            mock_dt.now.return_value = _make_utc(2026, 5, 1, 12, 0, 0)
            logger.log_row({"x": 1})

        day1_csv = log_dir / "test_2026-05-01.csv"
        assert day1_csv.exists()

        # Day 2 — triggers rotation
        with patch("src.csv_logger.datetime") as mock_dt:
            mock_dt.now.return_value = _make_utc(2026, 5, 2, 0, 0, 1)
            logger.log_row({"x": 2})

        # Day 1 file should now be compressed
        assert not day1_csv.exists()
        gz_path = log_dir / "test_2026-05-01.csv.gz"
        assert gz_path.exists()

        # Verify compressed content is valid CSV
        with gzip.open(gz_path, "rt") as f:
            rows = list(csv.reader(f))
        assert rows[0] == ["timestamp", "x"]
        assert rows[1][1] == "1"

        # Day 2 file is plain CSV
        day2_csv = log_dir / "test_2026-05-02.csv"
        assert day2_csv.exists()
        logger.close()

    def test_close_does_not_compress_current_file(self, log_dir: Path):
        """Closing the logger without rotation should NOT compress the file."""
        logger = CsvLogger(str(log_dir), "test", ["x"])
        logger.log_row({"x": 1})
        logger.close()

        csv_files = list(log_dir.glob("test_*.csv"))
        gz_files = list(log_dir.glob("test_*.csv.gz"))
        assert len(csv_files) == 1
        assert len(gz_files) == 0


from datetime import datetime, timezone


def _make_utc(y: int, m: int, d: int, h: int, mi: int, s: int) -> datetime:
    """Helper to create a timezone-aware UTC datetime."""
    return datetime(y, m, d, h, mi, s, tzinfo=timezone.utc)
