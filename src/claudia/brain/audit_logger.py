#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Audit Logging System - Track A Enhanced
For A/B testing decisions, canary observation, and fault rollback
"""

import json
import logging
import re
import threading
from pathlib import Path
from datetime import datetime
from typing import Optional, Dict, Any, List
from dataclasses import dataclass, asdict


@dataclass
class AuditEntry:
    """Audit log entry"""
    timestamp: str              # ISO format timestamp
    model_name: str             # Model name used
    input_command: str          # User input
    state_battery: Optional[float]  # Battery percentage
    state_standing: Optional[bool]  # Whether standing
    state_emergency: Optional[bool] # Whether in emergency state
    llm_output: Optional[str]   # Raw LLM output
    api_code: Optional[int]     # Final API code
    sequence: Optional[list]    # Action sequence
    safety_verdict: str         # Safety verdict: ok/rejected/modified
    safety_reason: Optional[str]  # Rejection/modification reason
    elapsed_ms: float           # Elapsed time (milliseconds)
    cache_hit: bool             # Whether cache was hit
    route: str                  # Route: emergency/cache/3B/7B
    success: bool               # Whether successful

    # PR2 extension fields (all Optional, default None -- backward compatible with old log deserialization)
    request_id: Optional[str] = None
    router_mode: Optional[str] = None       # "legacy" | "dual" | "shadow"
    shadow_comparison: Optional[Dict[str, Any]] = None
    action_latency_ms: Optional[float] = None
    voice_latency_ms: Optional[float] = None


def sanitize_audit_input(text, max_len=500):
    # type: (str, int) -> str
    """Sanitize audit log input

    - Replace embedded newlines with spaces (JSONL injection prevention)
    - Remove control characters (other than tab/space: 0x00-0x1F, 0x7F)
    - Truncate to max_len
    """
    if not isinstance(text, str):
        return str(text)[:max_len]
    text = text.replace("\n", " ").replace("\r", " ")
    text = re.sub(r"[\x00-\x08\x0b\x0c\x0e-\x1f\x7f]", "", text)
    return text[:max_len]


class AuditLogger:
    """Audit log manager"""

    def __init__(self, log_dir: str = "logs/audit", max_size_mb: int = 100):
        """
        Initialize audit logger

        Args:
            log_dir: Log directory
            max_size_mb: Maximum single log file size (MB)
        """
        self.log_dir = Path(log_dir).resolve()
        # Path traversal defense: ensure resolved path is a subdirectory of cwd
        # Uses parents set for checking, avoiding false matches with startswith
        # (e.g., cwd=/home/app, log_dir=/home/app-evil/... won't be allowed)
        _cwd = Path.cwd().resolve()
        if _cwd not in self.log_dir.parents and self.log_dir != _cwd:
            raise ValueError(
                "Audit log path {} is outside working directory {}".format(self.log_dir, _cwd)
            )
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.max_size_bytes = max_size_mb * 1024 * 1024

        # Get current log file
        self.current_log_file = self._get_current_log_file()

        # Write lock (protects JSONL line integrity in multi-threaded scenarios like run_in_executor)
        self._write_lock = threading.Lock()

        # Internal logger
        self.logger = logging.getLogger("AuditLogger")

    def _get_current_log_file(self) -> Path:
        """Get current log file path (daily rotation)"""
        date_str = datetime.now().strftime("%Y%m%d")
        log_file = self.log_dir / f"audit_{date_str}.jsonl"
        return log_file

    def log_entry(self, entry: AuditEntry) -> bool:
        """
        Record an audit entry (append to JSONL file)

        Args:
            entry: Audit entry

        Returns:
            True if write succeeded, False if write failed
        """
        try:
            with self._write_lock:
                # Check if log file needs rotation
                current_file = self._get_current_log_file()
                if current_file != self.current_log_file:
                    self.logger.info(f"Log file rotation: {self.current_log_file} -> {current_file}")
                    self.current_log_file = current_file

                # Check file size
                if self.current_log_file.exists():
                    if self.current_log_file.stat().st_size > self.max_size_bytes:
                        # Archive current file
                        archive_name = self.current_log_file.with_suffix(
                            f".{datetime.now().strftime('%H%M%S')}.jsonl"
                        )
                        self.current_log_file.rename(archive_name)
                        self.logger.warning(f"Log file too large, archived: {archive_name}")

                # Append JSONL entry
                with self.current_log_file.open('a', encoding='utf-8') as f:
                    json_line = json.dumps(asdict(entry), ensure_ascii=False)
                    f.write(json_line + '\n')
            return True

        except Exception as e:
            self.logger.error(f"Audit log write failed: {e}")
            return False

    def get_recent_entries(self, limit: int = 100) -> List[AuditEntry]:
        """
        Read recent audit entries

        Args:
            limit: Maximum number of entries to return

        Returns:
            List of audit entries
        """
        entries = []
        try:
            if not self.current_log_file.exists():
                return entries

            with self.current_log_file.open('r', encoding='utf-8') as f:
                lines = f.readlines()
                for line in lines[-limit:]:
                    data = json.loads(line.strip())
                    entries.append(AuditEntry(**data))
        except Exception as e:
            self.logger.error(f"Audit log read failed: {e}")

        return entries

    def get_stats(self, model_name: Optional[str] = None,
                  hours: int = 24) -> Dict[str, Any]:
        """
        Get statistical metrics (for A/B decision making)

        Args:
            model_name: Model name (None=all)
            hours: Statistics time window (hours)

        Returns:
            Statistics dictionary: {
                "total": Total requests,
                "success_rate": Success rate,
                "avg_latency_ms": Average latency,
                "p95_latency_ms": P95 latency,
                "safety_reject_rate": Safety rejection rate,
                "cache_hit_rate": Cache hit rate,
                ...
            }
        """
        entries = self.get_recent_entries(limit=10000)

        # Time filter
        cutoff = datetime.now().timestamp() - hours * 3600
        filtered = [
            e for e in entries
            if datetime.fromisoformat(e.timestamp).timestamp() > cutoff
        ]

        # Model filter
        if model_name:
            filtered = [e for e in filtered if e.model_name == model_name]

        if not filtered:
            return {"total": 0}

        # Calculate statistics
        total = len(filtered)
        success = sum(1 for e in filtered if e.success)
        rejected = sum(1 for e in filtered if e.safety_verdict == "rejected")
        cache_hits = sum(1 for e in filtered if e.cache_hit)
        latencies = sorted([e.elapsed_ms for e in filtered])

        return {
            "total": total,
            "success_rate": success / total if total > 0 else 0,
            "avg_latency_ms": sum(latencies) / len(latencies) if latencies else 0,
            "p50_latency_ms": latencies[len(latencies)//2] if latencies else 0,
            "p95_latency_ms": latencies[int(len(latencies)*0.95)] if latencies else 0,
            "p99_latency_ms": latencies[int(len(latencies)*0.99)] if latencies else 0,
            "safety_reject_rate": rejected / total if total > 0 else 0,
            "cache_hit_rate": cache_hits / total if total > 0 else 0,
        }


# Global singleton (thread-safe)
_global_audit_logger: Optional[AuditLogger] = None
_audit_logger_lock = threading.Lock()


def get_audit_logger(log_dir: str = "logs/audit") -> AuditLogger:
    """Get global audit logger (thread-safe: for run_in_executor scenarios)"""
    global _global_audit_logger
    if _global_audit_logger is None:
        with _audit_logger_lock:
            if _global_audit_logger is None:
                _global_audit_logger = AuditLogger(log_dir=log_dir)
    return _global_audit_logger
