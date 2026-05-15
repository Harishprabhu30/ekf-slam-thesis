# traj_cmd_v6_bright run notes

Run type: V2 lighting robustness master dataset
Scene: ekf-slam2-bright.usd
Lighting: bright
Trajectory method: manual fixed-rate teleop
Route protocol: square → straight → clockwise rotation → circular/curved motion

Command settings:
- Linear velocity: 0.41 m/s
- Angular velocity: 0.53 rad/s
- Teleop workflow: fixed-rate teleop

Physics settings used for V6:
- Floor static friction: 1.0
- Floor dynamic friction: 1.0
- Wheel material static friction: 0.6
- Wheel material dynamic friction: 0.6

Important note:
Exact V5 trajectory reproduction was tested but was not reliable in Isaac Sim.
This V6 run follows the same route protocol manually and is evaluated against its own /gt/odom.
This is not claimed as an identical repeat of V5.
