# X-Plane Feedback Control Project

This repo supports a controls project using X-Plane 12 as the high-fidelity simulator.
Architecture:
- XPPython3 plugin inside X-Plane streams telemetry over UDP and (later) accepts control commands.
- External Python app (WSL) logs, plots, computes metrics, and will run baseline + controller.

## Quick Start (Telemetry)
1) Deploy X-Plane Python plugins:
```bash
./scripts/deploy_to_xplane.sh