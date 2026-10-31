# D.9.6C.2E — Gazebo pose source discovery (read-only)

Scope: discover available Gazebo global pose topics and state services while sampling ROS `/clock` and `/{summit0,summit1,summit2}/odom`. This is *discovery*, not frame calibration. 0 publishers, 0 motion commands, no TF changes.

Install on host: `cd ~/swarmbots && unzip -o ~/Descargas/swarm_D9_6C_2E_descubrimiento_marcos.zip -d ~/swarmbots`

With Gazebo and bridge active in Docker:

```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/d96c2e_gazebo_probe.py --output scripts/resultados/d9_live/d96c2e_pose_probe.json
```

Share JSON report. `POSE_SOURCE_DISCOVERED_CALIBRATION_PENDING` indicates a possible global pose topic exists, not that frame equivalence was proven. If absent, next step is to inspect the Gazebo world configuration and enable a read-only pose source; no TF assumptions and no movement authorization.
