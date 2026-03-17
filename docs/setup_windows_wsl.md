# Setup Notes (Windows + WSL)

- X-Plane runs on Windows (installed at D:\X-Plane 12\).
- External Python runs in WSL.
- XPPython3 runs inside X-Plane and streams telemetry to WSL.

## WSL Drive Mount
Ensure D: is mounted:
```bash
sudo mkdir -p /mnt/d
sudo mount -t drvfs D: /mnt/d