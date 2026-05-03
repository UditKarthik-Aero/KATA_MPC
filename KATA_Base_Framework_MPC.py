import numpy as np
import matplotlib.pyplot as plt
import random

# 1. CONSTANTS & CONFIG

G = 9.81
F_MAX = 1_500_000.0
DRY_MASS = 25600.0
MAIN_FUEL_START = 22000.0    
RESERVE_FUEL_START = 2000.0  

WIND_STD = 15.0         
SENSOR_NOISE_STD = 0.5 
ISP_VAC, ISP_SEA = 380.0, 330.0  
TARGET_MAX_G = 4.5  

CD_SIDE, CD_TIP = 0.8, 0.2
AREA_SIDE, AREA_TIP = 45.0, 10.0
RHO0, H_SCALE = 1.225, 8500.0

def get_isp(y):
    rho_ratio = np.exp(-max(0, y) / H_SCALE)
    return ISP_VAC - (ISP_VAC - ISP_SEA) * rho_ratio

def get_aero_forces(vx, vy, y, theta):
    rho = RHO0 * np.exp(-max(0, y) / H_SCALE)
    v_vec = np.array([vx, vy]); v_mag = np.linalg.norm(v_vec)
    if v_mag < 0.1: return np.array([0.0, 0.0])
    v_hat = v_vec / v_mag
    rocket_axis = np.array([np.cos(theta), np.sin(theta)])
    aoa = np.arccos(np.clip(np.dot(v_hat, rocket_axis), -1, 1))
    sign = np.sign(v_hat[0]*rocket_axis[1] - v_hat[1]*rocket_axis[0])
    aoa = np.clip(aoa * sign, -0.35, 0.35)
    q = 0.5 * rho * v_mag**2
    CL = np.clip(2.0 * aoa, -0.8, 0.8)
    CD = CD_TIP + (CD_SIDE - CD_TIP) * (np.sin(abs(aoa))**2)
    area = AREA_TIP + (AREA_SIDE - AREA_TIP) * (np.sin(abs(aoa))**2)
    return (-q * CD * area * v_hat) + (q * CL * area * 0.3 * np.array([-v_hat[1], v_hat[0]]))

# 2. ENHANCED GUIDANCE

class LandingGuidance:
    def __init__(self):
        self.ignited = False

    def get_commands(self, x_obs, y_obs, vx_obs, vy_obs, current_mass):
        a_max = F_MAX / current_mass
        a_net = (min(a_max, TARGET_MAX_G * G)) - G
        burn_dist = (vy_obs**2) / (2 * max(0.1, a_net)) * 1.12

        if not self.ignited and y_obs > burn_dist:
            t_go = max(0.1, (vy_obs + np.sqrt(max(0, vy_obs**2 + 2*G*y_obs))) / G)
            target_vx = -(x_obs + vx_obs * t_go) / max(1.0, t_go)
            theta_aero = np.clip(0.05 * (target_vx - vx_obs), -0.35, 0.35)
            return 0.0, (np.pi/2) + theta_aero
        else:
            self.ignited = True
            if y_obs > 200: target_vy = -25.0
            elif y_obs > 50: target_vy = -8.0
            else: target_vy = -1.2  
            
            ay_des = 4.5 * (target_vy - vy_obs) + G
            t_impact = max(0.1, y_obs / max(0.1, abs(vy_obs)))
            ax_des = -3.0 * (x_obs + vx_obs * t_impact) / (t_impact**2)
            
            a_req = np.sqrt(ax_des**2 + ay_des**2)
            if a_req > TARGET_MAX_G * G:
                scale = (TARGET_MAX_G * G) / a_req
                ay_des *= scale; ax_des *= scale
            
            thrust = min(current_mass * np.sqrt(ax_des**2 + ay_des**2), F_MAX)
            angle = np.arctan2(ay_des, ax_des)
            return thrust, angle

# 3. MONTE CARLO WRAPPER

def run_single_sim():
    # Initial conditions with slight variance to mimic real flight profiles
    x, y, vx, vy = 4000.0, 25000.0, -140.0, -90.0
    main_f, res_f = MAIN_FUEL_START, RESERVE_FUEL_START
    theta, dt = np.pi/2, 0.01
    ctrl = LandingGuidance()
    
    while y > 0:
        # Sensor Noise Modeling
        x_obs = x + random.gauss(0, SENSOR_NOISE_STD)
        y_obs = y + random.gauss(0, SENSOR_NOISE_STD)
        vx_obs = vx + random.gauss(0, SENSOR_NOISE_STD * 0.1)
        vy_obs = vy + random.gauss(0, SENSOR_NOISE_STD * 0.1)

        m_total = DRY_MASS + main_f + res_f
        T_cmd, target_theta = ctrl.get_commands(x_obs, y_obs, vx_obs, vy_obs, m_total)

        fuel_req = (T_cmd / (get_isp(y) * G) * dt)
        actual_burnt = min(fuel_req, main_f + res_f)
        
        if main_f >= actual_burnt: main_f -= actual_burnt
        else:
            remainder = actual_burnt - main_f
            main_f = 0; res_f = max(0, res_f - remainder)
            
        T_actual = (actual_burnt / dt) * (get_isp(y) * G) if actual_burnt > 0 else 0
        aero = get_aero_forces(vx, vy, y, theta)
        
        # Actuator constraints
        theta += np.clip(((target_theta - theta + np.pi) % (2*np.pi) - np.pi), -0.3, 0.3)
        
        vx += ((T_actual * np.cos(theta) + aero[0]) / m_total) * dt
        vy += ((T_actual * np.sin(theta) + aero[1]) / m_total - G) * dt
        x += vx * dt; y += vy * dt

    return x, vx, vy, (main_f + res_f)

# 4. DATA AGGREGATION

NUM_RUNS = 1000
results = []

print(f"Starting Monte Carlo Simulation: {NUM_RUNS} iterations...")

for i in range(NUM_RUNS):
    res = run_single_sim()
    results.append(res)
    if (i+1) % 100 == 0:
        print(f"Progress: {i+1}/{NUM_RUNS} runs completed.")

# Convert to Numpy for Analysis
results = np.array(results)
final_offsets = np.abs(results[:, 0])
final_vxs = np.abs(results[:, 1])
final_vys = np.abs(results[:, 2])
final_fuels = results[:, 3]

# CRITERIA CHECK
success_mask = (final_vys < 5.0) & (final_vxs < 3.0) & (final_offsets < 2.0)
success_count = np.sum(success_mask)
pms = (success_count / NUM_RUNS) * 100

# 5. RESEARCH OUTPUTS

print("\n" + "="*40)
print(f"MONTE CARLO RESULTS (N={NUM_RUNS})")
print(f"Criteria: VY < 5m/s, VX < 3m/s, Offset < 2m")
print(f"PROBABILITY OF MISSION SUCCESS: {pms:.2f}%")
print("="*40)
print(f"Mean VY: {np.mean(final_vys):.4f} m/s")
print(f"Mean VX: {np.mean(final_vxs):.4f} m/s")
print(f"Mean Offset: {np.mean(final_offsets):.2f} m")
print(f"Mean Fuel Remaining: {np.mean(final_fuels):.2f} kg")
print("="*40)

# Histograms for the Research Paper
fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(18, 5))

ax1.hist(final_offsets, bins=30, color='blue', alpha=0.7)
ax1.axvline(2.0, color='red', linestyle='--', label='Threshold')
ax1.set_title("Landing Offset Distribution")
ax1.set_xlabel("Distance from Pad (m)")

ax2.hist(final_vys, bins=30, color='red', alpha=0.7)
ax2.axvline(5.0, color='black', linestyle='--', label='Threshold')
ax2.set_title("Vertical Velocity at Touchdown")
ax2.set_xlabel("VY (m/s)")

ax3.hist(final_vxs, bins=30, color='green', alpha=0.7)
ax3.axvline(3.0, color='black', linestyle='--', label='Threshold')
ax3.set_title("Horizontal Velocity at Touchdown")
ax3.set_xlabel("VX (m/s)")

plt.tight_layout()
plt.show()

# Detailed Failure Analysis
failed_indices = np.where(~success_mask)[0] # Finds indices of every failed run

v_fail = np.sum(final_vys[failed_indices] >= 5.0)
x_fail = np.sum(final_vxs[failed_indices] >= 3.0)
dist_fail = np.sum(final_offsets[failed_indices] >= 2.0)

print(f"Total Unique Failed Missions: {len(failed_indices)}")
print(f" - Failed on Vertical Velocity: {v_fail}")
print(f" - Failed on Horizontal Velocity: {x_fail}")
print(f" - Failed on Landing Offset: {dist_fail}")
