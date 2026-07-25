"""
process_lock.py — Single-Instance Process Lock Manager for ScanAR
===================================================================
Enforces resource isolation by claiming single-instance file locks in /tmp/
and killing any stale process holding the resource prior to node startup.
"""

import os
import sys
import time
import signal

class ProcessLock:
    def __init__(self, lock_name: str):
        self.lock_name = lock_name
        self.lock_file = f"/tmp/scanar_{lock_name}.lock"
        self._claim_lock()

    def _claim_lock(self):
        if os.path.exists(self.lock_file):
            try:
                with open(self.lock_file, "r") as f:
                    content = f.read().strip()
                    if content:
                        pid = int(content)
                        if pid != os.getpid() and self._is_pid_alive(pid):
                            print(f"[PROCESS LOCK] Killing stale process PID {pid} holding '{self.lock_name}' lock...")
                            try:
                                os.kill(pid, signal.SIGKILL)
                                time.sleep(0.2)
                            except OSError:
                                pass
            except Exception as e:
                print(f"[PROCESS LOCK] Warning inspecting lock {self.lock_file}: {e}")

        # Claim the lock file for current process PID
        try:
            with open(self.lock_file, "w") as f:
                f.write(str(os.getpid()))
        except Exception as e:
            print(f"[PROCESS LOCK] Error writing lock file {self.lock_file}: {e}")

    def release(self):
        try:
            if os.path.exists(self.lock_file):
                with open(self.lock_file, "r") as f:
                    pid = int(f.read().strip())
                if pid == os.getpid():
                    os.remove(self.lock_file)
        except Exception:
            pass

    @staticmethod
    def _is_pid_alive(pid: int) -> bool:
        if pid <= 0:
            return False
        try:
            os.kill(pid, 0)
            return True
        except OSError:
            return False
