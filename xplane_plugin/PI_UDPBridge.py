import json
import socket
import time
from XPPython3 import xp

# --- NETWORK CONFIG ---
TELEM_IP   = "172.26.100.110"   # WSL IP (update if WSL IP changes)
TELEM_PORT = 49005              # X-Plane -> WSL
CMD_PORT   = 49006              # WSL -> X-Plane
HZ = 50
# ----------------------

def clamp(x, lo=-1.0, hi=1.0):
    return max(lo, min(hi, x))

class PythonInterface:
    def XPluginStart(self):
        self.Name = "UDPBridge"
        self.Sig  = "team.udpbridge"
        self.Desc = "Telemetry + command bridge over UDP."

        xp.sys_log(f"[UDPBridge] START telem={TELEM_IP}:{TELEM_PORT} cmd=0.0.0.0:{CMD_PORT} Hz={HZ}")

        # Telemetry socket
        self.sock_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.dest = (TELEM_IP, TELEM_PORT)

        # Command socket (non-blocking)
        self.sock_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_rx.bind(("0.0.0.0", CMD_PORT))
        self.sock_rx.setblocking(False)

        # ---- Telemetry DataRefs (units per DataRefs.txt) ----
        self.dr_phi = xp.findDataRef("sim/flightmodel/position/phi")       # deg :contentReference[oaicite:4]{index=4}
        self.dr_p   = xp.findDataRef("sim/flightmodel/position/P")         # deg/s :contentReference[oaicite:5]{index=5}
        self.dr_psi = xp.findDataRef("sim/flightmodel/position/psi")       # deg :contentReference[oaicite:6]{index=6}
        self.dr_r   = xp.findDataRef("sim/flightmodel/position/R")         # deg/s :contentReference[oaicite:7]{index=7}
        self.dr_alt = xp.findDataRef("sim/flightmodel/position/elevation") # meters :contentReference[oaicite:8]{index=8}
        self.dr_vv  = xp.findDataRef("sim/flightmodel/position/vh_ind")    # m/s :contentReference[oaicite:9]{index=9}
        self.dr_ias = xp.findDataRef("sim/flightmodel/position/indicated_airspeed") # kias :contentReference[oaicite:10]{index=10}

        # ---- Control Input DataRefs (yoke ratios) ----
        self.dr_yoke_roll   = xp.findDataRef("sim/joystick/yoke_roll_ratio")     # [-1..1] :contentReference[oaicite:11]{index=11}
        self.dr_yoke_pitch  = xp.findDataRef("sim/joystick/yoke_pitch_ratio")    # [-1..1] :contentReference[oaicite:12]{index=12}
        self.dr_yoke_yaw    = xp.findDataRef("sim/joystick/yoke_heading_ratio")  # [-1..1] :contentReference[oaicite:13]{index=13}

        # ---- Overrides (so our yoke ratios actually drive the sim) ----
        self.dr_ovr_roll  = xp.findDataRef("sim/operation/override/override_joystick_roll")    # bool :contentReference[oaicite:14]{index=14}
        self.dr_ovr_pitch = xp.findDataRef("sim/operation/override/override_joystick_pitch")   # bool :contentReference[oaicite:15]{index=15}
        self.dr_ovr_yaw   = xp.findDataRef("sim/operation/override/override_joystick_heading") # bool :contentReference[oaicite:16]{index=16}

        # Command state
        self.cmd_enable = 0
        self.cmd_roll = 0.0
        self.cmd_pitch = 0.0
        self.cmd_yaw = 0.0
        self.last_cmd_t = 0.0
        self.cmd_timeout_s = 0.5  # safety: if commands stop, release override

        # Flightloop
        self.flightloop_id = xp.createFlightLoop(
            (xp.FlightLoop_Phase_AfterFlightModel, self._cb, None)
        )
        xp.scheduleFlightLoop(self.flightloop_id, 1.0 / HZ, 1)
        xp.sys_log("[UDPBridge] Flightloop scheduled")

        return self.Name, self.Sig, self.Desc

    def XPluginEnable(self):
        xp.sys_log("[UDPBridge] ENABLED")
        return 1

    def XPluginDisable(self):
        xp.sys_log("[UDPBridge] DISABLED")
        self._release_controls()

    def XPluginStop(self):
        xp.sys_log("[UDPBridge] STOPPED")
        self._release_controls()
        try: self.sock_tx.close()
        except: pass
        try: self.sock_rx.close()
        except: pass

    def XPluginReceiveMessage(self, inFromWho, inMessage, inParam):
        pass

    # ----- Control helpers -----
    def _apply_controls(self):
        # Enable overrides
        xp.setDatai(self.dr_ovr_roll,  1)
        xp.setDatai(self.dr_ovr_pitch, 1)
        xp.setDatai(self.dr_ovr_yaw,   1)

        # Apply yoke commands [-1..1]
        xp.setDataf(self.dr_yoke_roll,  float(self.cmd_roll))
        xp.setDataf(self.dr_yoke_pitch, float(self.cmd_pitch))
        xp.setDataf(self.dr_yoke_yaw,   float(self.cmd_yaw))

    def _release_controls(self):
        # Neutralize commands
        try:
            xp.setDataf(self.dr_yoke_roll,  0.0)
            xp.setDataf(self.dr_yoke_pitch, 0.0)
            xp.setDataf(self.dr_yoke_yaw,   0.0)
        except:
            pass

        # Release overrides (return control to user)
        try:
            xp.setDatai(self.dr_ovr_roll,  0)
            xp.setDatai(self.dr_ovr_pitch, 0)
            xp.setDatai(self.dr_ovr_yaw,   0)
        except:
            pass

    def _poll_commands(self):
        # Drain UDP socket (keep last packet)
        got = False
        while True:
            try:
                data, addr = self.sock_rx.recvfrom(4096)
            except BlockingIOError:
                break

            try:
                msg = json.loads(data.decode("utf-8"))
            except Exception:
                continue

            # expected keys: enable, roll, pitch, yaw
            if "enable" in msg: self.cmd_enable = int(msg["enable"])
            if "roll" in msg:   self.cmd_roll = clamp(float(msg["roll"]))
            if "pitch" in msg:  self.cmd_pitch = clamp(float(msg["pitch"]))
            if "yaw" in msg:    self.cmd_yaw = clamp(float(msg["yaw"]))

            self.last_cmd_t = time.time()
            got = True

        return got

    # ----- Main loop -----
    def _cb(self, *_args):
        now = time.time()

        # 1) command handling
        self._poll_commands()

        # safety timeout: if enabled but no commands recently, release controls
        if self.cmd_enable == 1 and (now - self.last_cmd_t) > self.cmd_timeout_s:
            xp.sys_log("[UDPBridge] Command timeout -> releasing controls")
            self.cmd_enable = 0
            self._release_controls()

        if self.cmd_enable == 1:
            self._apply_controls()
        else:
            # don’t constantly force neutral if the user is flying;
            # just ensure overrides are released
            self._release_controls()

        # 2) telemetry
        try:
            telem = {
                "t": now,
                "phi_deg": float(xp.getDataf(self.dr_phi)),
                "p_deg_s": float(xp.getDataf(self.dr_p)),
                "psi_deg": float(xp.getDataf(self.dr_psi)),
                "r_deg_s": float(xp.getDataf(self.dr_r)),
                "alt_m": float(xp.getDataf(self.dr_alt)),
                "vv_m_s": float(xp.getDataf(self.dr_vv)),
                "ias_kts": float(xp.getDataf(self.dr_ias)),
                "cmd_enable": int(self.cmd_enable),
                "cmd_roll": float(self.cmd_roll),
                "cmd_pitch": float(self.cmd_pitch),
                "cmd_yaw": float(self.cmd_yaw),
            }
            self.sock_tx.sendto(json.dumps(telem).encode("utf-8"), self.dest)
        except Exception as e:
            xp.sys_log(f"[UDPBridge] Telemetry send error: {repr(e)}")

        return 1.0 / HZ