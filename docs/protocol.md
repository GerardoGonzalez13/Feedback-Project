# UDP Protocol

## Ports
- Telemetry: X-Plane -> WSL : UDP 49005
- Commands:  WSL -> X-Plane : UDP 49006 (reserved for later)

## Telemetry message (JSON, UTF-8)
Sent at ~50 Hz.

Keys and units:
- t: unix timestamp (float seconds)
- phi_deg: bank angle [deg]
- p_deg_s: roll rate [deg/s]
- psi_deg: heading/yaw angle [deg]
- r_deg_s: yaw rate [deg/s]
- alt_m: altitude MSL [m]
- vv_m_s: vertical speed [m/s]
- ias_kts: indicated airspeed [kt/s]

Example:
```json
{"t":123.45,"phi_rad":0.1,"p_rad_s":0.02,"psi_rad":1.2,"r_rad_s":-0.01,"alt_m":950.2,"vv_m_s":0.3,"ias_m_s":50.0}


