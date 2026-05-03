# KATA_MPC


# KATA-MPC: Phased Predictive Aerospace Controller

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python 3.8+](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)

**KATA-MPC** (Kinematic Adaptive Trajectory Algorithm - Model Predictive Controller) is a high-fidelity landing simulation designed to solve the non-linear control problem of autonomous rocket recovery. 

By utilizing a three-phase guidance transition—moving from aerodynamic glide-slope correction to a multi-stage throttled landing burn—this model achieves high-precision landings under stochastic environmental disturbances.

## 🚀 Research Highlights
*   **Probability of Mission Success (PMS):** 97.00% (Validated via N=1000 Monte Carlo Simulation).
*   **Target Precision:** Mean landing offset of ~0.20m.
*   **Robustness:** Demonstrated stability under 15 m/s Gaussian wind gusts and sensor noise.
*   **Physics Engine:** Features altitude-dependent atmospheric density, variable $I_{sp}$ models, and non-linear Lift/Drag coefficients.

## 📊 Performance Analysis
<p align="center">
  <img src="path/to/your/monte_carlo_results.png" width="800" alt="Monte Carlo Results">
  <br><i>Figure 1: Distribution of landing offsets and touchdown velocities across 1,000 iterations.</i>
</p>

## 🛠 Methodology
The controller operates in three distinct phases:
1.  **Aero-Glide Phase:** Correcting lateral drift using atmospheric lift and angle of attack (AoA) optimization.
2.  **Pulsed Transition:** Calculating the optimal "Suicide Burn" ignition point based on real-time mass-altitude-velocity (MAV) data.
3.  **Final Powered Descent:** A high-frequency feedback loop to minimize touchdown velocity ($V_y < 5m/s$) and horizontal drift ($V_x < 3m/s$).

## 💻 Installation & Usage
```bash
# Clone the repository
git clone [https://github.com/yourusername/KATA-MPC.git](https://github.com/yourusername/KATA-MPC.git)

# Install dependencies
pip install numpy matplotlib

# Run the Monte Carlo Simulation
python rocket_sim.py
