import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('TkAgg')

import numpy as np
from scipy.interpolate import interp1d
from scipy.optimize import curve_fit

# Input data
aoa = np.array([0, 30, 60, 90])  # Angle of attack in degrees
Fx = np.array([0.4, 11.6, 15.8, 18])  # Force in x-direction (relative to plate)
Fy = np.array([12, 7.4, 4.4, -1.8])  # Force in y-direction (relative to plate)

# --- Convert forces to fixed world axis (AoA = 0 as reference) ---
aoa_rad = np.deg2rad(aoa)
Fx_world = Fx * np.cos(aoa_rad) + Fy * np.sin(aoa_rad)
Fy_world = Fx * np.sin(aoa_rad) - Fy * np.cos(aoa_rad)

# --- Interpolation for smoother curves (optional but helpful for visualization) ---
fx_interp = interp1d(aoa, Fx_world, kind='cubic')
fy_interp = interp1d(aoa, Fy_world, kind='cubic')
aoa_fine = np.linspace(0, 90, 500)
Fx_world_interp = fx_interp(aoa_fine)
Fy_world_interp = fy_interp(aoa_fine)

# --- Granular RFT Model Fitting ---

# RFT Model Functions (for world frame) - Modified to include constant terms
def rft_Fx(alpha, Cl_alpha, Cd0, Cd_alpha, Cd_alpha2, aspect_ratio, Fx_const):
    alpha_rad = np.deg2rad(alpha)
    Cl = Cl_alpha * alpha_rad
    Cd = Cd0 + Cd_alpha * alpha_rad + Cd_alpha2 * alpha_rad**2
    Cd_i = Cl**2 / (np.pi * aspect_ratio)
    Cd_total = Cd + Cd_i
    Fx = Cd_total * np.cos(alpha_rad) - Cl * np.sin(alpha_rad) + Fx_const
    return Fx

def rft_Fy(alpha, Cl_alpha, Cd0, Cd_alpha, Cd_alpha2, aspect_ratio, Fy_const):
    alpha_rad = np.deg2rad(alpha)
    Cl = Cl_alpha * alpha_rad
    Cd = Cd0 + Cd_alpha * alpha_rad + Cd_alpha2 * alpha_rad**2
    Cd_i = Cl**2 / (np.pi * aspect_ratio)
    Cd_total = Cd + Cd_i
    Fy = Cd_total * np.sin(alpha_rad) + Cl * np.cos(alpha_rad) + Fy_const
    return Fy

# Initial parameter guesses (you'll need to adjust these)
initial_guesses = [0.1, 0.02, 0.001, 0.0002, 2, 0]  # Reduced to 6
param_bounds = ([0, 0, -np.inf, -np.inf, 0, -np.inf], [np.inf, np.inf, np.inf, np.inf, np.inf, np.inf]) # Reduced to 6
from scipy.optimize import OptimizeWarning
import warnings
# Fit the RFT model to Fx_world
try:
    # Fit the RFT model to Fx_world
    popt_Fx, pcov_Fx = curve_fit(rft_Fx, aoa, Fx_world, p0=initial_guesses, bounds=param_bounds)

    # Fit the RFT model to Fy_world
    popt_Fy, pcov_Fy = curve_fit(rft_Fy, aoa, Fy_world, p0=initial_guesses, bounds=param_bounds)

except RuntimeError as e:
    print(f"Error during curve fitting: {e}")
    # Handle the error (e.g., set default values for popt_Fx, pcov_Fx, etc.)

except OptimizeWarning as w:
    print(f"Warning during curve fitting: {w}")
    warnings.warn("Curve fitting may not have converged.", OptimizeWarning)
    # Handle the warning if needed
# Evaluate the fitted RFT model
Fx_world_fit = rft_Fx(aoa_fine, *popt_Fx)  # Pass all the parameters from popt_Fx
Fy_world_fit = rft_Fy(aoa_fine, *popt_Fy)  # Pass all the parameters from popt_Fy

# --- Plotting ---
plt.figure(figsize=(12, 10))

# --- Plot 1: Original Data (Fx vs AoA) ---
plt.subplot(2, 2, 1)
plt.plot(aoa, Fx, 'ro-', label='Original Data (Fx)')
plt.xlabel('Angle of Attack (degrees)')
plt.ylabel('Force (N)')
plt.title('Forces in Plate Frame (Fx)')
plt.grid(True)
plt.legend()

# --- Plot 2: Original Data (Fy vs AoA) ---
plt.subplot(2, 2, 3)
plt.plot(aoa, Fy, 'bo-', label='Original Data (Fy)')
plt.xlabel('Angle of Attack (degrees)')
plt.ylabel('Force (N)')
plt.title('Forces in Plate Frame (Fy)')
plt.grid(True)
plt.legend()

# --- Plot 3: World Frame Data (Fx_world vs AoA) ---
plt.subplot(2, 2, 2)
plt.plot(aoa, Fx_world, 'ro', label='Original Data (Fx_world)')
plt.plot(aoa_fine, Fx_world_interp, 'r-', label='Interpolated (Fx_world)')
plt.plot(aoa_fine, Fx_world_fit, 'g--', label='RFT Fit (Fx_world)')
plt.xlabel('Angle of Attack (degrees)')
plt.ylabel('Force (N)')
plt.title('Forces in World Frame (Fx)')
plt.grid(True)
plt.legend()

# --- Plot 4: World Frame Data (Fy_world vs AoA) ---
plt.subplot(2, 2, 4)
plt.plot(aoa, Fy_world, 'bo', label='Original Data (Fy_world)')
plt.plot(aoa_fine, Fy_world_interp, 'b-', label='Interpolated (Fy_world)')
plt.plot(aoa_fine, Fy_world_fit, 'g--', label='RFT Fit (Fy_world)')
plt.xlabel('Angle of Attack (degrees)')
plt.ylabel('Force (N)')
plt.title('Forces in World Frame (Fy)')
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()

# --- Print fitted parameters ---
print("Fitted RFT Parameters (Fx):")
print(f"  Cl_alpha: {popt_Fx[0]:.4f}")
print(f"  Cd0: {popt_Fx[1]:.4f}")
print(f"  Cd_alpha: {popt_Fx[2]:.4f}")
print(f"  Cd_alpha2: {popt_Fx[3]:.4f}")
print(f"  Aspect Ratio: {popt_Fx[4]:.4f}")

print("\nFitted RFT Parameters (Fy):")
print(f"  Cl_alpha: {popt_Fy[0]:.4f}")
print(f"  Cd0: {popt_Fy[1]:.4f}")
print(f"  Cd_alpha: {popt_Fy[2]:.4f}")
print(f"  Cd_alpha2: {popt_Fy[3]:.4f}")
print(f"  Aspect Ratio: {popt_Fy[4]:.4f}")