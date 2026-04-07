import csv
import json
import math
import os
import socket
import time
from pathlib import Path
from typing import Optional, Dict, Any

from run_controlled import (
    TARGET_BANK_DEG as CONTROLLED_TARGET_BANK_DEG,
    BANK_CAPTURE_MIN_DEG,
    BANK_CAPTURE_MAX_DEG,
    BANK_CAPTURE_MAX_P_DEG_S,
    BANK_CAPTURE_DWELL_S,
    BANK_CAPTURE_TIMEOUT_S,
    K_TO_TARGET,
    P_DAMP,
    UMAX_TO_TARGET,
    CAPTURE_MODE_START_DEG,
    CAPTURE_K_TO_TARGET,
    CAPTURE_P_DAMP,
    CAPTURE_UMAX,
    MAX_ROLL_CMD_STEP_CAPTURE,
    capture_condition_met,
)

# =========================
# Network / X-Plane bridge
# =========================
WINDOWS_HOST_IP = "172.26.96.1"
STATE_PORT = int(os.environ.get("XPLANE_STATE_PORT", "49005"))
CMD_PORT = int(os.environ.get("XPLANE_CMD_PORT", "49006"))
RUNS_DIR = Path("data/runs")

RATE_HZ = 25
DT = 1.0 / RATE_HZ

# =========================
# Baseline maneuver settings
# =========================
TARGET_BANK_DEG = CONTROLLED_TARGET_BANK_DEG
MAX_ENTRY_TIME = BANK_CAPTURE_TIMEOUT_S
OBS_WINDOW_AFTER_BANK60_S = 20.0

# Optional mild pitch help during entry/hold so altitude does not get destroyed
# This is NOT a real altitude controller, just enough to stop the worst of the drop.
USE_MILD_PITCH_SUPPORT = True
MAX_PITCH_CMD = 0.08
KH_ALT = 0.0010                  # pitch command per foot altitude error
KV_VS = 0.010                    # pitch command per m/s vertical speed damping

# Safety / completion
FINAL_NEUTRAL_TIME = 0.5         # send neutral briefly before releasing override


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def smoothstep01(x: float) -> float:
    x = clamp(x, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)


def rate_limit(new_cmd: float, prev_cmd: float, max_step: float) -> float:
    delta = clamp(new_cmd - prev_cmd, -max_step, max_step)
    return prev_cmd + delta


class XPlaneBaselineRunner:
    def __init__(self) -> None:
        RUNS_DIR.mkdir(parents=True, exist_ok=True)
        self.out_path = RUNS_DIR / f"baseline_{int(time.time())}.csv"
        self.rows = []
        self.last_cmd = {"enable": 0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
        self.last_roll_cmd = 0.0
        self.phase = "init"
        self.first_bank60_time = None

        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.state_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.state_sock.bind(("0.0.0.0", STATE_PORT))
        except OSError as exc:
            if exc.errno == 98:
                raise RuntimeError(
                    f"Telemetry port {STATE_PORT} is already in use. "
                    "Another local process is likely already listening on that UDP port "
                    "(for example external/logger.py, external/run_compare.py, or "
                    "external/controller_roll.py). Stop the other listener, or rerun "
                    "with XPLANE_STATE_PORT set to the telemetry port you want to use."
                ) from exc
            raise
        self.state_sock.settimeout(0.25)

    def send_cmd(self, enable: int, roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0) -> None:
        msg = {
            "enable": int(enable),
            "roll": float(roll),
            "pitch": float(pitch),
            "yaw": float(yaw),
        }
        self.last_cmd = dict(msg)
        self.last_roll_cmd = msg["roll"]
        self.cmd_sock.sendto(json.dumps(msg).encode("utf-8"), (WINDOWS_HOST_IP, CMD_PORT))

    def recv_state(self) -> Optional[Dict[str, Any]]:
        try:
            data, _ = self.state_sock.recvfrom(65535)
            msg = json.loads(data.decode("utf-8"))
            self.record_state(msg)
            return msg
        except socket.timeout:
            return None
        except (json.JSONDecodeError, UnicodeDecodeError):
            return None

    def wait_for_fresh_state(self, timeout_s: float = 5.0) -> Dict[str, Any]:
        t0 = time.time()
        last = None
        while time.time() - t0 < timeout_s:
            msg = self.recv_state()
            if msg is not None:
                last = msg
                if "phi_deg" in msg and "p_deg_s" in msg:
                    return msg
        raise RuntimeError("Did not receive valid telemetry on STATE_PORT.")

    def mild_pitch_support(self, state: Dict[str, Any], alt_ref_ft: float) -> float:
        if not USE_MILD_PITCH_SUPPORT:
            return 0.0

        alt_ft = self.altitude_ft(state)
        vv_m_s = float(state.get("vv_m_s", 0.0))  # positive = climbing

        alt_err_ft = alt_ref_ft - alt_ft
        pitch_cmd = KH_ALT * alt_err_ft - KV_VS * vv_m_s
        return clamp(pitch_cmd, -MAX_PITCH_CMD, MAX_PITCH_CMD)

    def altitude_ft(self, state: Dict[str, Any]) -> float:
        if "alt_ft" in state:
            return float(state["alt_ft"])
        if "altitude_ft" in state:
            return float(state["altitude_ft"])
        if "alt_m" in state:
            return float(state["alt_m"]) * 3.28084
        return 0.0

    def record_state(self, state: Dict[str, Any]) -> None:
        row = dict(state)
        row.setdefault("cmd_enable", self.last_cmd["enable"])
        row.setdefault("cmd_roll", self.last_cmd["roll"])
        row.setdefault("cmd_pitch", self.last_cmd["pitch"])
        row.setdefault("cmd_yaw", self.last_cmd["yaw"])
        row["baseline_phase"] = self.phase
        self.rows.append(row)

    def flush_log(self) -> None:
        if not self.rows:
            return

        keys = sorted({k for row in self.rows for k in row.keys()})
        with self.out_path.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=keys)
            writer.writeheader()
            writer.writerows(self.rows)
        print(f"[baseline] Wrote {len(self.rows)} telemetry rows -> {self.out_path}")

    def bank_hold_roll_cmd(self, state: Dict[str, Any], target_bank_deg: float) -> float:
        phi = float(state["phi_deg"])
        p = float(state["p_deg_s"])
        bank_err = target_bank_deg - phi
        if phi < CAPTURE_MODE_START_DEG:
            raw_roll_cmd = (K_TO_TARGET * bank_err) - (P_DAMP * p)
            raw_roll_cmd = clamp(raw_roll_cmd, -UMAX_TO_TARGET, UMAX_TO_TARGET)
        else:
            raw_roll_cmd = (CAPTURE_K_TO_TARGET * bank_err) - (CAPTURE_P_DAMP * p)
            raw_roll_cmd = clamp(raw_roll_cmd, -CAPTURE_UMAX, CAPTURE_UMAX)

        return rate_limit(raw_roll_cmd, self.last_roll_cmd, MAX_ROLL_CMD_STEP_CAPTURE)

    def hold_fixed_cmd(self, duration_s: float, enable: int, roll: float, pitch: float, yaw: float) -> None:
        t_end = time.time() + duration_s
        while time.time() < t_end:
            self.send_cmd(enable=enable, roll=roll, pitch=pitch, yaw=yaw)
            time.sleep(DT)

    def acquire_target_bank(self, target_bank_deg: float, alt_ref_ft: float) -> Dict[str, Any]:
        self.phase = "acquire_bank"
        print(f"[baseline] Acquiring target bank: {target_bank_deg:.1f} deg")
        t_start = time.time()
        capture_candidate_start = None
        last_state = None

        while time.time() - t_start < MAX_ENTRY_TIME:
            state = self.recv_state()
            if state is None:
                self.send_cmd(enable=1, roll=0.0, pitch=0.0, yaw=0.0)
                time.sleep(DT)
                continue

            last_state = state
            phi = float(state["phi_deg"])
            p = float(state["p_deg_s"])
            now = time.time()
            elapsed_s = now - t_start

            roll_cmd = self.bank_hold_roll_cmd(state, target_bank_deg)
            pitch_cmd = self.mild_pitch_support(state, alt_ref_ft)

            self.send_cmd(enable=1, roll=roll_cmd, pitch=pitch_cmd, yaw=0.0)

            if capture_condition_met(phi, p):
                if capture_candidate_start is None:
                    capture_candidate_start = now
            else:
                capture_candidate_start = None

            if capture_candidate_start is not None and (now - capture_candidate_start) >= BANK_CAPTURE_DWELL_S:
                self.first_bank60_time = now
                print(
                    f"[baseline] Handoff triggered: status=clean_capture "
                    f"t={elapsed_s:.2f}s phi={phi:.2f} deg p={p:.2f} deg/s"
                )
                return state

            time.sleep(DT)

        if last_state is None:
            print(
                f"[baseline] Handoff triggered: status=capture_failed "
                f"t={MAX_ENTRY_TIME:.2f}s phi=nan deg p=nan deg/s"
            )
            raise RuntimeError("Timed out acquiring target bank and telemetry was lost.")

        last_phi = float(last_state["phi_deg"])
        last_p = float(last_state["p_deg_s"])
        elapsed_s = time.time() - t_start
        print(
            f"[baseline] Handoff triggered: status=capture_failed "
            f"t={elapsed_s:.2f}s phi={last_phi:.2f} deg p={last_p:.2f} deg/s"
        )
        raise RuntimeError(
            "Stage 1 failed to achieve clean capture before timeout. "
            f"Required phi in [{BANK_CAPTURE_MIN_DEG:.1f}, {BANK_CAPTURE_MAX_DEG:.1f}] deg, "
            f"|p| <= {BANK_CAPTURE_MAX_P_DEG_S:.1f} deg/s, held for {BANK_CAPTURE_DWELL_S:.1f} s "
            f"within {MAX_ENTRY_TIME:.1f} s. "
            f"Final state at timeout: phi={last_phi:.2f} deg, p={last_p:.2f} deg/s."
        )

    def neutral_observation_window_until(self, end_time_s: float) -> None:
        self.phase = "neutral_observation"
        remaining = max(0.0, end_time_s - time.time())
        print(f"[baseline] Neutral-command observation window: {remaining:.1f}s remaining")
        t_end = end_time_s
        while time.time() < t_end:
            # Keep draining telemetry so the raw CSV captures the full
            # post-bank response instead of ending at the last active-hold phase.
            self.recv_state()
            self.send_cmd(enable=1, roll=0.0, pitch=0.0, yaw=0.0)
            time.sleep(DT)

    def run(self) -> None:
        try:
            self.phase = "wait_for_telemetry"
            print("[baseline] Waiting for telemetry...")
            state0 = self.wait_for_fresh_state(timeout_s=5.0)

            # Use the initial altitude as a mild reference during acquisition / pulse phases.
            alt_ref_ft = self.altitude_ft(state0)
            print(f"[baseline] Initial altitude reference: {alt_ref_ft:.1f} ft")

            # 0) Take control and settle at neutral.
            self.phase = "neutral_settle"
            print("[baseline] Settling at neutral with override ON")
            self.hold_fixed_cmd(duration_s=1.0, enable=1, roll=0.0, pitch=0.0, yaw=0.0)

            # 1) Acquire ~60 deg bank using the same Stage 1 roll-in / clean
            # capture logic as the controlled run.
            self.acquire_target_bank(target_bank_deg=TARGET_BANK_DEG, alt_ref_ft=alt_ref_ft)

            # 2) Baseline window: once clean capture is achieved, switch
            # immediately to neutral-command observation.
            if self.first_bank60_time is None:
                self.first_bank60_time = time.time()
                print("[baseline] WARNING: clean-capture event was not captured explicitly; using current time.")
            obs_end_time = self.first_bank60_time + OBS_WINDOW_AFTER_BANK60_S
            print(
                f"[baseline] Post-bank observation window: {OBS_WINDOW_AFTER_BANK60_S:.1f}s "
                "after clean capture"
            )
            self.neutral_observation_window_until(end_time_s=obs_end_time)

            # 3) Brief neutral flush, then release override.
            self.phase = "release"
            print("[baseline] Finishing run, releasing control")
            self.hold_fixed_cmd(duration_s=FINAL_NEUTRAL_TIME, enable=1, roll=0.0, pitch=0.0, yaw=0.0)
            self.send_cmd(enable=0)

            print("[baseline] Done.")

        finally:
            try:
                self.send_cmd(enable=0)
            except Exception:
                pass
            self.phase = "complete"
            self.flush_log()
            self.cmd_sock.close()
            self.state_sock.close()


if __name__ == "__main__":
    runner = XPlaneBaselineRunner()
    runner.run()
