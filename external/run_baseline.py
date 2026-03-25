import csv
import json
import math
import os
import socket
import time
from pathlib import Path
from typing import Optional, Dict, Any

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
TARGET_BANK_DEG = 60.0
# Practical capture thresholds for the baseline maneuver. These are intended
# to confirm a usable, settled bank capture before the yaw pulse, not to act
# as ultra-tight controller-performance metrics.
BANK_TOL_DEG = 3.0
ROLLRATE_STABLE_THRESH = 1.0     # deg/s
STABLE_HOLD_TIME = 0.3           # must remain near target this long before yaw pulse
ROLLRATE_PULSE_THRESH = 1.0      # deg/s
CAPTURE_GRACE_TIME = 0.75        # allow extra time to complete dwell if we are already near capture
BANK_RESET_TOL_DEG = 3.5         # relaxed reset threshold to avoid one-sample timer resets
ROLLRATE_RESET_THRESH = 1.2      # relaxed reset threshold to avoid one-sample timer resets

MAX_ENTRY_TIME = 20.0            # seconds allowed to get to target bank
RUDDER_PULSE_TIME = 0.4          # seconds
POST_PULSE_HOLD_TIME = 0.6       # keep bank stabilized briefly after pulse
OBS_WINDOW_AFTER_BANK60_S = 20.0 # keep logging this long after first reaching ~60 deg bank

# Roll-command schedule / damping
MAX_ROLL_CMD = 0.18              # reduce authority to lower overshoot into capture
KP_BANK = 0.018                  # command per degree of bank error
KD_P = 0.040                     # stronger roll-rate damping so p is arrested sooner
CAPTURE_MODE_BANK_DEG = 55.0
TARGET_WINDOW_DEG = 2.0
KP_CAPTURE = 0.010               # keep some bank-error correction in capture mode
KD_P_CAPTURE = 0.070             # prioritize killing roll rate near 60 deg
MAX_CAPTURE_CMD_STEP = 0.015     # limit per-cycle command change near capture
MAX_CAPTURE_ROLL_CMD = 0.12      # softer authority near the target to avoid spikes
KD_P_CAPTURE_NEAR = 0.090        # extra damping once we are in the final target window
TARGET_WINDOW_BRAKE_CMD = 0.020  # small smooth braking bias while positive p remains
KP_CAPTURE_OUTSIDE = 0.020       # pull bank angle back more decisively when outside 58-62 deg
MAX_CAPTURE_ROLL_CMD_OUTSIDE = 0.15
MIN_TARGET_WINDOW_BRAKE = 0.035  # keep a small steady brake while positive p remains near target
TARGET_WINDOW_HOLD_BRAKE = 0.045 # hold a slightly stronger brake until the final settle is complete

# Optional mild pitch help during entry/hold so altitude does not get destroyed
# This is NOT a real altitude controller, just enough to stop the worst of the drop.
USE_MILD_PITCH_SUPPORT = True
MAX_PITCH_CMD = 0.08
KH_ALT = 0.0010                  # pitch command per foot altitude error
KV_VS = 0.010                    # pitch command per m/s vertical speed damping

# Rudder pulse
YAW_PULSE_CMD = 0.20

# Safety / completion
FINAL_NEUTRAL_TIME = 0.5         # send neutral briefly before releasing override


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def smoothstep01(x: float) -> float:
    x = clamp(x, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)


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
        cmd = KP_BANK * bank_err - KD_P * p

        # Start tapering earlier so the entry command fades before we drift
        # through 60 deg with substantial positive roll rate.
        abs_err = abs(bank_err)
        if abs_err < 25.0:
            cmd *= 0.80
        if abs_err < 15.0:
            cmd *= 0.65
        if abs_err < 8.0:
            cmd *= 0.50

        # Capture mode: as we approach 60 deg, blend away from entry behavior
        # and toward a smoother roll-rate arrest law.
        if phi >= CAPTURE_MODE_BANK_DEG:
            capture_blend = smoothstep01((phi - CAPTURE_MODE_BANK_DEG) / (target_bank_deg - CAPTURE_MODE_BANK_DEG))
            kd_capture = KD_P_CAPTURE
            kp_capture = KP_CAPTURE
            capture_roll_limit = MAX_CAPTURE_ROLL_CMD
            if abs(bank_err) <= TARGET_WINDOW_DEG:
                kd_capture = KD_P_CAPTURE_NEAR
            else:
                kp_capture = KP_CAPTURE_OUTSIDE
                capture_roll_limit = MAX_CAPTURE_ROLL_CMD_OUTSIDE

            raw_capture_cmd = kp_capture * bank_err - kd_capture * p

            # In the final 58-62 deg window, keep a small braking bias while
            # p is still positive so the controller does not release too early
            # and let roll rate creep back up.
            if abs(bank_err) <= TARGET_WINDOW_DEG and p > ROLLRATE_PULSE_THRESH:
                positive_p_blend = smoothstep01(
                    (p - ROLLRATE_PULSE_THRESH) / (1.5 - ROLLRATE_PULSE_THRESH)
                )
                raw_capture_cmd -= TARGET_WINDOW_BRAKE_CMD * positive_p_blend

            # In the final target window, taper more gently and keep a small
            # negative hold command while p is still positive so the aircraft
            # does not re-accelerate in roll before the gate is satisfied.
            if abs(bank_err) <= TARGET_WINDOW_DEG and abs(p) < 0.8:
                raw_capture_cmd *= 0.85
            if abs(bank_err) <= TARGET_WINDOW_DEG and abs(p) < 0.35:
                raw_capture_cmd *= 0.75
            if abs(bank_err) <= TARGET_WINDOW_DEG and 0.25 < p < 0.8:
                raw_capture_cmd = min(raw_capture_cmd, -TARGET_WINDOW_HOLD_BRAKE)
            elif abs(bank_err) <= TARGET_WINDOW_DEG and 0.0 < p <= 0.25:
                raw_capture_cmd = min(raw_capture_cmd, -MIN_TARGET_WINDOW_BRAKE)

            raw_capture_cmd = clamp(raw_capture_cmd, -capture_roll_limit, capture_roll_limit)

            # Blend smoothly from the tapered entry command into the capture
            # command so the handoff above 55 deg does not create a kink.
            blended_capture_cmd = (1.0 - capture_blend) * cmd + capture_blend * raw_capture_cmd

            # Rate-limit the command in capture mode to prevent abrupt sign
            # flips and the large one-step negative correction seen near target.
            delta = clamp(
                blended_capture_cmd - self.last_roll_cmd,
                -MAX_CAPTURE_CMD_STEP,
                MAX_CAPTURE_CMD_STEP,
            )
            cmd = self.last_roll_cmd + delta

            return clamp(cmd, -capture_roll_limit, capture_roll_limit)

        return clamp(cmd, -MAX_ROLL_CMD, MAX_ROLL_CMD)

    def hold_fixed_cmd(self, duration_s: float, enable: int, roll: float, pitch: float, yaw: float) -> None:
        t_end = time.time() + duration_s
        while time.time() < t_end:
            self.send_cmd(enable=enable, roll=roll, pitch=pitch, yaw=yaw)
            time.sleep(DT)

    def acquire_target_bank(self, target_bank_deg: float, alt_ref_ft: float) -> Dict[str, Any]:
        self.phase = "acquire_bank"
        print(f"[baseline] Acquiring target bank: {target_bank_deg:.1f} deg")
        t_start = time.time()
        t_deadline = t_start + MAX_ENTRY_TIME
        stable_start = None
        last_state = None
        in_grace_period = False

        while True:
            state = self.recv_state()
            if state is None:
                # Keep previous command cadence alive even if telemetry blips
                if time.time() >= t_deadline:
                    break
                time.sleep(DT)
                continue

            last_state = state
            phi = float(state["phi_deg"])
            p = float(state["p_deg_s"])
            now = time.time()

            roll_cmd = self.bank_hold_roll_cmd(state, target_bank_deg)
            pitch_cmd = self.mild_pitch_support(state, alt_ref_ft)

            self.send_cmd(enable=1, roll=roll_cmd, pitch=pitch_cmd, yaw=0.0)

            near_bank = abs(phi - target_bank_deg) <= BANK_TOL_DEG
            low_rollrate = abs(p) <= ROLLRATE_PULSE_THRESH
            hard_out_of_window = (
                abs(phi - target_bank_deg) > BANK_RESET_TOL_DEG
                or abs(p) > ROLLRATE_RESET_THRESH
            )

            if self.first_bank60_time is None and near_bank:
                self.first_bank60_time = now
                print(
                    f"[baseline] First reached ~60 deg bank at +{self.first_bank60_time - t_start:.2f}s "
                    f"(phi={phi:.2f} deg, p={p:.2f} deg/s)"
                )

            if near_bank and low_rollrate:
                if stable_start is None:
                    stable_start = now
                elif now - stable_start >= STABLE_HOLD_TIME:
                    print(f"[baseline] Target bank acquired: phi={phi:.2f} deg, p={p:.2f} deg/s")
                    return state
            elif hard_out_of_window:
                stable_start = None

            if now >= t_deadline:
                if stable_start is not None and not in_grace_period:
                    t_deadline = now + CAPTURE_GRACE_TIME
                    in_grace_period = True
                else:
                    break

            time.sleep(DT)

        if last_state is None:
            raise RuntimeError("Timed out acquiring target bank and telemetry was lost.")

        last_phi = float(last_state["phi_deg"])
        last_p = float(last_state["p_deg_s"])
        if self.first_bank60_time is None and abs(last_phi - target_bank_deg) <= BANK_TOL_DEG:
            self.first_bank60_time = time.time()
            print(
                f"[baseline] First reached ~60 deg bank on final capture sample "
                f"(phi={last_phi:.2f} deg, p={last_p:.2f} deg/s)"
            )
        if abs(last_phi - target_bank_deg) <= BANK_TOL_DEG and abs(last_p) <= ROLLRATE_PULSE_THRESH:
            print(f"[baseline] Target bank accepted on final state: phi={last_phi:.2f} deg, p={last_p:.2f} deg/s")
            return last_state

        raise RuntimeError(
            f"Timed out acquiring target bank. Last state: "
            f"phi={last_state.get('phi_deg')}, p={last_state.get('p_deg_s')}"
        )

    def pulse_rudder_while_holding_bank(self, target_bank_deg: float, alt_ref_ft: float) -> None:
        self.phase = "yaw_pulse"
        print(f"[baseline] Applying yaw pulse: {YAW_PULSE_CMD:.2f} for {RUDDER_PULSE_TIME:.2f}s")
        t_end = time.time() + RUDDER_PULSE_TIME
        while time.time() < t_end:
            state = self.recv_state()
            if state is None:
                time.sleep(DT)
                continue

            roll_cmd = self.bank_hold_roll_cmd(state, target_bank_deg)
            pitch_cmd = self.mild_pitch_support(state, alt_ref_ft)
            self.send_cmd(enable=1, roll=roll_cmd, pitch=pitch_cmd, yaw=YAW_PULSE_CMD)
            time.sleep(DT)

        self.phase = "post_pulse_hold"
        print(f"[baseline] Brief post-pulse bank hold: {POST_PULSE_HOLD_TIME:.2f}s")
        t_end = time.time() + POST_PULSE_HOLD_TIME
        while time.time() < t_end:
            state = self.recv_state()
            if state is None:
                time.sleep(DT)
                continue

            roll_cmd = self.bank_hold_roll_cmd(state, target_bank_deg)
            pitch_cmd = self.mild_pitch_support(state, alt_ref_ft)
            self.send_cmd(enable=1, roll=roll_cmd, pitch=pitch_cmd, yaw=0.0)
            time.sleep(DT)

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

            # 1) Acquire ~60 deg bank using telemetry-based entry logic.
            self.acquire_target_bank(target_bank_deg=TARGET_BANK_DEG, alt_ref_ft=alt_ref_ft)

            # 2) Apply rudder pulse only once target bank is actually reached.
            self.pulse_rudder_while_holding_bank(target_bank_deg=TARGET_BANK_DEG, alt_ref_ft=alt_ref_ft)

            # 3) Baseline window: once ~60 deg bank is first reached, keep
            # logging neutral-command response for a fixed post-bank window.
            if self.first_bank60_time is None:
                self.first_bank60_time = time.time()
                print("[baseline] WARNING: bank-60 event was not captured explicitly; using current time.")
            obs_end_time = self.first_bank60_time + OBS_WINDOW_AFTER_BANK60_S
            print(
                f"[baseline] Post-bank observation window: {OBS_WINDOW_AFTER_BANK60_S:.1f}s "
                "after first reaching ~60 deg bank"
            )
            self.neutral_observation_window_until(end_time_s=obs_end_time)

            # 4) Brief neutral flush, then release override.
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
