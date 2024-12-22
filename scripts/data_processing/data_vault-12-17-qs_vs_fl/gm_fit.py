import matplotlib.pyplot as plt
import matplotlib
import numpy as np
from scipy.interpolate import interp1d
from scipy.optimize import curve_fit, OptimizeWarning
import warnings
import sympy

matplotlib.use('TkAgg')

# Input data
aoa = np.array([0, 30, 60, 90])  # Angle of attack in degrees
Fx = np.array([0.4, 11.6, 15.8, 18])  # Force in x-direction (relative to plate)
Fy = np.array([12, 7.4, 4.4, -1.8])  # Force in y-direction (relative to plate)

Fx_lf = np.array([0.6, 9.16, 11.9, 13.3])  # Force in x-direction (low-friction case)
Fy_lf = np.array([7.04, 4.36, 1.9, 0.3])  # Force in y-direction (low-friction case)

x_ratio = abs(Fx_lf/Fx)*100
y_ratio = abs(Fy_lf/Fy)*100

x_amount = abs(Fx_lf - Fx)
y_amount = abs(Fy_lf - Fy)

# --- Convert forces to fixed world axis (AoA = 0 as reference) ---
aoa_rad = np.deg2rad(aoa)
Fx_world = Fx * np.cos(aoa_rad) - Fy * np.sin(aoa_rad)
Fy_world = Fx * np.sin(aoa_rad) + Fy * np.cos(aoa_rad)

Fx_lf_world = Fx_lf * np.cos(aoa_rad) - Fy_lf * np.sin(aoa_rad)
Fy_lf_world = Fx_lf * np.sin(aoa_rad) + Fy_lf * np.cos(aoa_rad)

# --- Interpolation for smoother curves (optional) ---
aoa_fine = np.linspace(0, 90, 500)

# --- Granular RFT Model Fitting ---

# RFT Model Functions (for world frame)
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

# Initial parameter guesses
initial_guesses = [0.1, 0.02, 0.001, 0.0002, 2, 0]
param_bounds = ([0, 0, -np.inf, -np.inf, 0, -np.inf], [np.inf, np.inf, np.inf, np.inf, np.inf, np.inf])

# Fit the RFT model to (Fx, Fy)
try:
    popt_Fx, pcov_Fx = curve_fit(rft_Fx, aoa, Fx_world, p0=initial_guesses, bounds=param_bounds)
    popt_Fy, pcov_Fy = curve_fit(rft_Fy, aoa, Fy_world, p0=initial_guesses, bounds=param_bounds)
except (RuntimeError, OptimizeWarning) as e:
    print(f"Error during curve fitting: {e}")

# Fit the RFT model to (Fx_lf, Fy_lf)
try:
    popt_Fx_lf, pcov_Fx_lf = curve_fit(rft_Fx, aoa, Fx_lf_world, p0=initial_guesses, bounds=param_bounds)
    popt_Fy_lf, pcov_Fy_lf = curve_fit(rft_Fy, aoa, Fy_lf_world, p0=initial_guesses, bounds=param_bounds)
except (RuntimeError, OptimizeWarning) as e:
    print(f"Error during curve fitting: {e}")

# Evaluate the fitted RFT models
Fx_world_fit = rft_Fx(aoa_fine, *popt_Fx)
Fy_world_fit = rft_Fy(aoa_fine, *popt_Fy)

Fx_lf_world_fit = rft_Fx(aoa_fine, *popt_Fx_lf)
Fy_lf_world_fit = rft_Fy(aoa_fine, *popt_Fy_lf)

# --- Plotting ---
plt.figure(figsize=(3.5, 3.5 * 0.8))

font_size = 10
plt.rcParams['font.family'] = 'Arial'
plt.rcParams['font.size'] = font_size
# plt.rcParams['line.']

# Plot (Fx, Fy) vs aoa
plt.plot(aoa, Fx_world, 'ro', label='$F_x$')
plt.plot(aoa_fine, Fx_world_fit, 'r-', label='RFT Fit $F_x$')
plt.plot(aoa, Fy_world, 'bv', label='$F_y$')
plt.plot(aoa_fine, Fy_world_fit, 'b-', label='RFT Fit $F_y$')

# Plot (Fx_lf, Fy_lf) vs aoa
plt.plot(aoa, Fx_lf_world, 'rs', markerfacecolor='none', label='$F_{x,lf}$')
plt.plot(aoa_fine, Fx_lf_world_fit, 'r--', label='RFT Fit $F_{x,lf}$')
plt.plot(aoa, Fy_lf_world, 'b^', markerfacecolor='none', label='$F_{y,lf}$')
plt.plot(aoa_fine, Fy_lf_world_fit, 'b--', label='RFT Fit $F_{y,lf}$')

plt.xlabel('Angle of attack ($^{\circ}$)')
plt.ylabel('Force (N)')
plt.title('Drag force of intruder')
plt.grid(False)
plt.xlim([-5, 95])
plt.ylim([-9, 19])
plt.xticks([0, 30, 60, 90])
plt.rcParams['figure.figsize'] = (3.5, 3.5/0.8)
plt.show()
# plt.legend()
plt.tight_layout()
plt.savefig("RFT_fit.pdf", format='pdf', dpi=600)



# --- Print fitted parameters ---
print("Fitted RFT Parameters (Fx, Fy):")
print(f"  Cl_alpha: {popt_Fx[0]:.4f}")
print(f"  Cd0: {popt_Fx[1]:.4f}")
print(f"  Cd_alpha: {popt_Fx[2]:.4f}")
print(f"  Cd_alpha2: {popt_Fx[3]:.4f}")
print(f"  Aspect Ratio: {popt_Fx[4]:.4f}")
print(f"  Fx_const: {popt_Fx[5]:.4f}")

print("\nFitted RFT Parameters (Fx_lf, Fy_lf):")
print(f"  Cl_alpha: {popt_Fx_lf[0]:.4f}")
print(f"  Cd0: {popt_Fx_lf[1]:.4f}")
print(f"  Cd_alpha: {popt_Fx_lf[2]:.4f}")
print(f"  Cd_alpha2: {popt_Fx_lf[3]:.4f}")
print(f"  Aspect Ratio: {popt_Fx_lf[4]:.4f}")
print(f"  Fx_const: {popt_Fx_lf[5]:.4f}")

# --- Symbolic calculations using sympy ---
alpha, Cl_alpha, Cd0, Cd_alpha, Cd_alpha2, AR, Fx_const, Fy_const = sympy.symbols(
    'alpha Cl_alpha Cd0 Cd_alpha Cd_alpha2 AR Fx_const Fy_const'
)
Cl = Cl_alpha * alpha
Cd = Cd0 + Cd_alpha * alpha + Cd_alpha2 * alpha ** 2
Cd_i = Cl ** 2 / (sympy.pi * AR)
Cd_total = Cd + Cd_i

Fx_eqn = sympy.Eq(sympy.Symbol('Fx'), Cd_total * sympy.cos(alpha) - Cl * sympy.sin(alpha) + Fx_const)
Fy_eqn = sympy.Eq(sympy.Symbol('Fy'), Cd_total * sympy.sin(alpha) + Cl * sympy.cos(alpha) + Fy_const)

# Substitute the fitted values for (Fx, Fy)
Fx_eqn_fitted = Fx_eqn.subs({Cl_alpha: popt_Fx[0], Cd0: popt_Fx[1], Cd_alpha: popt_Fx[2],
                             Cd_alpha2: popt_Fx[3], AR: popt_Fx[4], Fx_const: popt_Fx[5]})
Fy_eqn_fitted = Fy_eqn.subs({Cl_alpha: popt_Fy[0], Cd0: popt_Fy[1], Cd_alpha: popt_Fy[2],
                             Cd_alpha2: popt_Fy[3], AR: popt_Fy[4], Fy_const: popt_Fy[5]})

# Substitute the fitted values for (Fx_lf, Fy_lf)
Fx_lf_eqn_fitted = Fx_eqn.subs({Cl_alpha: popt_Fx_lf[0], Cd0: popt_Fx_lf[1], Cd_alpha: popt_Fx_lf[2],
                                Cd_alpha2: popt_Fx_lf[3], AR: popt_Fx_lf[4], Fx_const: popt_Fx_lf[5]})
Fy_lf_eqn_fitted = Fy_eqn.subs({Cl_alpha: popt_Fy_lf[0], Cd0: popt_Fy_lf[1], Cd_alpha: popt_Fy_lf[2],
                                Cd_alpha2: popt_Fy_lf[3], AR: popt_Fy_lf[4], Fy_const: popt_Fy_lf[5]})

print("\nFitted RFT Equations (Fx, Fy):")
sympy.pprint(Fx_eqn_fitted)
sympy.pprint(Fy_eqn_fitted)

print("\nFitted RFT Equations (Fx_lf, Fy_lf):")
sympy.pprint(Fx_lf_eqn_fitted)
sympy.pprint(Fy_lf_eqn_fitted)


# Raw data

