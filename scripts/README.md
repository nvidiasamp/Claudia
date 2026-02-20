# Scripts

Utility scripts for deployment, testing, validation, and maintenance.

## Directory Structure

```
scripts/
  setup/            # Environment and dependency installation
  deploy/           # Production deployment scripts
  maintenance/      # Log cleanup and housekeeping
  validation/       # Hardware sensor validation suites
    audio/          # USB microphone and ASR validation
    camera/         # Front camera validation
    foot_force/     # Foot force sensor validation
    imu/            # IMU sensor validation
  test/             # Legacy test scripts (use test/ directory instead)
```

## Key Scripts

| Script | Purpose |
|--------|---------|
| `setup/setup_cyclonedds.sh` | CycloneDDS build and install |
| `setup/install_unitree_sdks.sh` | Unitree SDK2 Python setup |
| `setup/setup_ros2_environment.sh` | ROS2 Foxy environment config |
| `deploy/start_production_brain_v2.sh` | Deployment launcher with nohup |
| `maintenance/daily_cleanup.sh` | Log rotation and temp cleanup |
| `offline_route_comparison.py` | Dual vs legacy route comparison (offline) |
| `shadow_observation_commands.py` | Shadow mode A/B observation runner |
| `smoke_test_three_modes.py` | Quick smoke test for all routing modes |
| `setup_asr_venv.sh` | ASR Python 3.11 venv setup |

## Validation Suites

Hardware sensor validation scripts from initial robot bring-up. Each suite has its own README.
Run with robot connected via ethernet (`192.168.123.x`).
