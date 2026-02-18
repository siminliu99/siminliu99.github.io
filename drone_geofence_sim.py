"""
3D Drone Geofence Simulation with Safe Controller (CBF-style)
=============================================================
Visualizes a quadrotor-like drone inside a transparent geofence box.
A wind gust pushes the drone toward a wall; a safety controller (barrier-
function inspired) activates to keep it inside the boundary.

Requirements:  conda activate drone_sim   (or pip install mujoco numpy)
Run:           mjpython drone_geofence_sim.py   (macOS requires mjpython, not python)
"""

import mujoco
import mujoco.viewer
import numpy as np
import time

# ── Simulation parameters ────────────────────────────────────────────
SIM_DURATION = 18.0
FENCE_HALF = 4.0
FENCE_HEIGHT = 4.0
DRONE_MASS = 1.0

# Barrier / safe-controller parameters
BARRIER_MARGIN = 1.2
BARRIER_GAIN = 25.0
DAMPING = 3.0

# Wind parameters
WIND_START = 4.0
WIND_PEAK = 7.0
WIND_END = 13.0
WIND_DIR = np.array([1.0, 0.3, 0.0])
WIND_DIR /= np.linalg.norm(WIND_DIR)
WIND_STRENGTH = 18.0

# Home position
HOME = np.array([0.0, 0.0, 2.0])

# ── Helpers ───────────────────────────────────────────────────────────

def wind_force(t):
    """Smooth wind gust envelope (ramp up, hold, ramp down)."""
    if t < WIND_START:
        return np.zeros(3)
    elif t < WIND_PEAK:
        alpha = (t - WIND_START) / (WIND_PEAK - WIND_START)
        return WIND_STRENGTH * alpha * WIND_DIR
    elif t < WIND_END:
        alpha = 1.0 - (t - WIND_PEAK) / (WIND_END - WIND_PEAK)
        return WIND_STRENGTH * alpha * WIND_DIR
    else:
        return np.zeros(3)


def barrier_control(pos, vel):
    """
    CBF-inspired safe controller.  For each axis, if the drone is within
    BARRIER_MARGIN of a fence wall AND moving toward it, apply a repulsive
    force proportional to proximity and add velocity damping.
    """
    force = np.zeros(3)
    upper_limits = np.array([FENCE_HALF, FENCE_HALF, FENCE_HEIGHT])
    lower_limits = np.array([-FENCE_HALF, -FENCE_HALF, 0.2])

    for i in range(3):
        dist_upper = upper_limits[i] - pos[i]
        if dist_upper < BARRIER_MARGIN:
            penetration = max(0.0, BARRIER_MARGIN - dist_upper)
            force[i] -= BARRIER_GAIN * penetration
            if vel[i] > 0:
                force[i] -= DAMPING * vel[i]

        dist_lower = pos[i] - lower_limits[i]
        if dist_lower < BARRIER_MARGIN:
            penetration = max(0.0, BARRIER_MARGIN - dist_lower)
            force[i] += BARRIER_GAIN * penetration
            if vel[i] < 0:
                force[i] -= DAMPING * vel[i]

    return force


def is_barrier_active(pos):
    upper_limits = np.array([FENCE_HALF, FENCE_HALF, FENCE_HEIGHT])
    lower_limits = np.array([-FENCE_HALF, -FENCE_HALF, 0.2])
    for i in range(3):
        if (upper_limits[i] - pos[i]) < BARRIER_MARGIN:
            return True
        if (pos[i] - lower_limits[i]) < BARRIER_MARGIN:
            return True
    return False


# ── MuJoCo XML model ─────────────────────────────────────────────────
H = FENCE_HALF
Z = FENCE_HEIGHT
WALL_THICK = 0.02
EDGE_RAD = 0.03

# Build wind arrow geoms (3x3 grid, will be toggled via rgba alpha)
N_ARROWS = 9
arrow_geoms_xml = ""
for idx in range(N_ARROWS):
    arrow_geoms_xml += f"""
        <geom name="wind_arrow_{idx}" type="cylinder" size="0.015 0.5"
              rgba="0.2 0.7 1.0 0.0" euler="0 90 0"/>
        <geom name="wind_head_{idx}" type="box" size="0.06 0.06 0.015"
              rgba="0.2 0.7 1.0 0.0" euler="0 45 0"/>
    """

# Trail: pre-allocate small spheres for the flight trail
N_TRAIL = 200
trail_geoms_xml = ""
for idx in range(N_TRAIL):
    trail_geoms_xml += f"""
        <geom name="trail_{idx}" type="sphere" size="0.04"
              rgba="0.1 0.8 0.3 0.0" pos="0 0 -10"/>
    """

XML = f"""
<mujoco model="drone_geofence">
  <option timestep="0.005" gravity="0 0 -9.81" integrator="RK4"/>

  <visual>
    <global offwidth="1280" offheight="960"/>
    <rgba fog="0.9 0.9 0.95 1"/>
    <quality shadowsize="2048"/>
  </visual>

  <asset>
    <texture type="2d" name="grid_tex" builtin="checker" rgb1="0.85 0.85 0.85"
             rgb2="0.75 0.75 0.75" width="100" height="100"/>
    <material name="grid_mat" texture="grid_tex" texrepeat="8 8" reflectance="0.1"/>
    <material name="fence_mat" rgba="1.0 0.3 0.2 0.12" reflectance="0.0"/>
    <material name="fence_edge_mat" rgba="1.0 0.35 0.25 0.9"/>
    <material name="drone_body_mat" rgba="0.15 0.15 0.15 1.0"/>
    <material name="drone_arm_mat" rgba="0.35 0.35 0.35 1.0"/>
    <material name="rotor_mat" rgba="0.1 0.55 0.85 0.75"/>
  </asset>

  <worldbody>
    <!-- Ground -->
    <geom type="plane" size="10 10 0.1" material="grid_mat"/>
    <light pos="0 0 8" dir="0 0 -1" diffuse="0.8 0.8 0.8" specular="0.3 0.3 0.3"/>
    <light pos="5 5 6" dir="-1 -1 -1" diffuse="0.4 0.4 0.4"/>

    <!-- ── Geofence walls (translucent) ─────────────────────────── -->
    <!-- +X wall -->
    <geom type="box" size="{WALL_THICK} {H} {Z/2}" pos="{H} 0 {Z/2}" material="fence_mat"/>
    <!-- -X wall -->
    <geom type="box" size="{WALL_THICK} {H} {Z/2}" pos="{-H} 0 {Z/2}" material="fence_mat"/>
    <!-- +Y wall -->
    <geom type="box" size="{H} {WALL_THICK} {Z/2}" pos="0 {H} {Z/2}" material="fence_mat"/>
    <!-- -Y wall -->
    <geom type="box" size="{H} {WALL_THICK} {Z/2}" pos="0 {-H} {Z/2}" material="fence_mat"/>
    <!-- Top -->
    <geom type="box" size="{H} {H} {WALL_THICK}" pos="0 0 {Z}" material="fence_mat"/>

    <!-- ── Geofence edges (bright wireframe look) ───────────────── -->
    <!-- Bottom edges -->
    <geom type="capsule" fromto="{-H} {-H} 0 {H} {-H} 0" size="{EDGE_RAD}" material="fence_edge_mat"/>
    <geom type="capsule" fromto="{H} {-H} 0 {H} {H} 0" size="{EDGE_RAD}" material="fence_edge_mat"/>
    <geom type="capsule" fromto="{H} {H} 0 {-H} {H} 0" size="{EDGE_RAD}" material="fence_edge_mat"/>
    <geom type="capsule" fromto="{-H} {H} 0 {-H} {-H} 0" size="{EDGE_RAD}" material="fence_edge_mat"/>
    <!-- Top edges -->
    <geom type="capsule" fromto="{-H} {-H} {Z} {H} {-H} {Z}" size="{EDGE_RAD}" material="fence_edge_mat"/>
    <geom type="capsule" fromto="{H} {-H} {Z} {H} {H} {Z}" size="{EDGE_RAD}" material="fence_edge_mat"/>
    <geom type="capsule" fromto="{H} {H} {Z} {-H} {H} {Z}" size="{EDGE_RAD}" material="fence_edge_mat"/>
    <geom type="capsule" fromto="{-H} {H} {Z} {-H} {-H} {Z}" size="{EDGE_RAD}" material="fence_edge_mat"/>
    <!-- Vertical edges -->
    <geom type="capsule" fromto="{-H} {-H} 0 {-H} {-H} {Z}" size="{EDGE_RAD}" material="fence_edge_mat"/>
    <geom type="capsule" fromto="{H} {-H} 0 {H} {-H} {Z}" size="{EDGE_RAD}" material="fence_edge_mat"/>
    <geom type="capsule" fromto="{H} {H} 0 {H} {H} {Z}" size="{EDGE_RAD}" material="fence_edge_mat"/>
    <geom type="capsule" fromto="{-H} {H} 0 {-H} {H} {Z}" size="{EDGE_RAD}" material="fence_edge_mat"/>

    <!-- ── Drone (free body) ────────────────────────────────────── -->
    <body name="drone" pos="0 0 2">
      <freejoint name="drone_joint"/>
      <inertial pos="0 0 0" mass="{DRONE_MASS}" diaginertia="0.01 0.01 0.02"/>
      <!-- Central body -->
      <geom type="box" size="0.15 0.15 0.05" material="drone_body_mat" contype="0" conaffinity="0"/>
      <!-- Arms -->
      <geom type="capsule" fromto="-0.28 -0.28 0 0.28 0.28 0" size="0.02" material="drone_arm_mat" contype="0" conaffinity="0"/>
      <geom type="capsule" fromto="0.28 -0.28 0 -0.28 0.28 0" size="0.02" material="drone_arm_mat" contype="0" conaffinity="0"/>
      <!-- Rotors -->
      <geom type="cylinder" pos="0.28 0.28 0.05" size="0.13 0.008" material="rotor_mat" contype="0" conaffinity="0"/>
      <geom type="cylinder" pos="0.28 -0.28 0.05" size="0.13 0.008" material="rotor_mat" contype="0" conaffinity="0"/>
      <geom type="cylinder" pos="-0.28 0.28 0.05" size="0.13 0.008" material="rotor_mat" contype="0" conaffinity="0"/>
      <geom type="cylinder" pos="-0.28 -0.28 0.05" size="0.13 0.008" material="rotor_mat" contype="0" conaffinity="0"/>
    </body>

    <!-- ── Wind arrow geoms & trail (dynamic decoration) ────────── -->
    {arrow_geoms_xml}
    {trail_geoms_xml}
  </worldbody>

  <actuator>
    <!-- Direct force actuators on the drone free joint (tx, ty, tz) -->
    <general joint="drone_joint" ctrlrange="-50 50" gear="1 0 0 0 0 0" name="fx"/>
    <general joint="drone_joint" ctrlrange="-50 50" gear="0 1 0 0 0 0" name="fy"/>
    <general joint="drone_joint" ctrlrange="-50 50" gear="0 0 1 0 0 0" name="fz"/>
    <!-- Torque actuators for attitude stabilization -->
    <general joint="drone_joint" ctrlrange="-5 5" gear="0 0 0 1 0 0" name="tx"/>
    <general joint="drone_joint" ctrlrange="-5 5" gear="0 0 0 0 1 0" name="ty"/>
    <general joint="drone_joint" ctrlrange="-5 5" gear="0 0 0 0 0 1" name="tz"/>
  </actuator>
</mujoco>
"""

# ── Load model and create viewer ──────────────────────────────────────
model = mujoco.MjModel.from_xml_string(XML)
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)

# Lookup geom IDs for dynamic visuals
arrow_geom_ids = []
arrow_head_ids = []
for idx in range(N_ARROWS):
    arrow_geom_ids.append(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"wind_arrow_{idx}"))
    arrow_head_ids.append(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"wind_head_{idx}"))

trail_geom_ids = []
for idx in range(N_TRAIL):
    trail_geom_ids.append(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"trail_{idx}"))

drone_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "drone")

# Trail state
trail_idx = 0
trail_timer = 0.0
TRAIL_INTERVAL = 0.08  # seconds between trail dots

print("=" * 60)
print("  Drone Geofence Simulation — Safe Controller Demo")
print("=" * 60)
print("Phases:")
print("  0-4s   Normal hovering flight")
print("  4-13s  Wind gust pushes drone toward +X fence wall")
print("         Safe controller activates near the boundary")
print("  13-18s Wind subsides, drone recovers to center")
print()
print("Controls: Drag to orbit, scroll to zoom, double-click to track")
print("=" * 60)

with mujoco.viewer.launch_passive(model, data) as viewer:
    # Set initial camera
    viewer.cam.azimuth = 135
    viewer.cam.elevation = -25
    viewer.cam.distance = 14
    viewer.cam.lookat[:] = [0, 0, 2]

    t = 0.0
    while viewer.is_running() and t < SIM_DURATION:
        step_start = time.time()

        # Get drone state
        # For a freejoint: qpos = [x,y,z, qw,qx,qy,qz], qvel = [vx,vy,vz, wx,wy,wz]
        pos = data.qpos[:3].copy()
        quat = data.qpos[3:7].copy()
        vel = data.qvel[:3].copy()
        ang_vel = data.qvel[3:6].copy()

        # ── Compute forces ──────────────────────────────────────────
        # 1. Hover (cancel gravity)
        hover = np.array([0, 0, 9.81 * DRONE_MASS])

        # 2. Gentle PD hold toward home
        pos_err = HOME - pos
        hold = 1.5 * pos_err - 1.0 * vel

        # 3. Wind
        w = wind_force(t)

        # 4. Safe controller
        b = barrier_control(pos, vel)
        barrier_on = is_barrier_active(pos)

        total = hover + hold + w + b

        # 5. Attitude stabilization: convert quaternion to euler-ish errors
        # Simple approach: use the rotation matrix's deviation from identity
        rot = np.zeros(9)
        mujoco.mju_quat2Mat(rot, quat)
        rot = rot.reshape(3, 3)
        # Roll/pitch errors from off-diagonal elements
        roll_err = rot[2, 1]   # ~sin(roll)
        pitch_err = -rot[2, 0]  # ~sin(pitch)
        yaw_err = rot[1, 0]    # ~sin(yaw)

        torque = np.array([
            -15 * roll_err  - 3 * ang_vel[0],
            -15 * pitch_err - 3 * ang_vel[1],
            -2 * yaw_err    - 1 * ang_vel[2],
        ])

        # Apply via actuators (clipped to ctrlrange)
        data.ctrl[0] = np.clip(total[0], -50, 50)
        data.ctrl[1] = np.clip(total[1], -50, 50)
        data.ctrl[2] = np.clip(total[2], -50, 50)
        data.ctrl[3] = np.clip(torque[0], -5, 5)
        data.ctrl[4] = np.clip(torque[1], -5, 5)
        data.ctrl[5] = np.clip(torque[2], -5, 5)

        # ── Update visual decorations ───────────────────────────────
        w_mag = np.linalg.norm(w)

        # Wind arrows: 3x3 grid positioned upstream of the drone
        grid_offsets = [(-1.2, -0.8), (-1.2, 0.0), (-1.2, 0.8),
                        (0.0, -0.8),  (0.0, 0.0),  (0.0, 0.8),
                        (1.2, -0.8),  (1.2, 0.0),  (1.2, 0.8)]
        for idx in range(N_ARROWS):
            gid = arrow_geom_ids[idx]
            hid = arrow_head_ids[idx]
            if w_mag > 0.5:
                dy, dz = grid_offsets[idx]
                arrow_center = pos + np.array([-2.0, dy, dz])
                arrow_len = w_mag / WIND_STRENGTH * 1.2
                alpha = min(1.0, w_mag / WIND_STRENGTH) * 0.85

                model.geom_pos[gid] = arrow_center
                model.geom_size[gid] = [0.015, arrow_len / 2, 0]
                model.geom_rgba[gid] = [0.2, 0.55 + 0.4 * alpha, 1.0, alpha]

                # Arrowhead at the tip
                head_pos = arrow_center + WIND_DIR * arrow_len / 2
                model.geom_pos[hid] = head_pos
                model.geom_rgba[hid] = [0.2, 0.55 + 0.4 * alpha, 1.0, alpha]
            else:
                model.geom_rgba[gid] = [0, 0, 0, 0]
                model.geom_rgba[hid] = [0, 0, 0, 0]

        # Trail dots
        trail_timer += model.opt.timestep
        if trail_timer >= TRAIL_INTERVAL:
            trail_timer = 0.0
            gid = trail_geom_ids[trail_idx % N_TRAIL]
            model.geom_pos[gid] = pos.copy()
            if barrier_on:
                model.geom_rgba[gid] = [1.0, 0.85, 0.1, 0.9]  # yellow when barrier active
            else:
                model.geom_rgba[gid] = [0.1, 0.8, 0.3, 0.9]   # green normal
            trail_idx += 1

        # Step physics
        mujoco.mj_step(model, data)
        t = data.time

        # Sync viewer
        viewer.sync()

        # Real-time pacing
        elapsed = time.time() - step_start
        sleep_time = model.opt.timestep - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

print("\nSimulation complete!")
