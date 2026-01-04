import numpy as np
import matplotlib.pyplot as plt

def cauchy_influence(s, c=1.0):
    return (2 * s) / (1 + (s / c)**2)

def huber_influence(s, delta=1.0):
    return np.where(np.abs(s) <= delta, s, delta * np.sign(s))

# Parameters
s = np.linspace(-30, 30, 600)
c_cauchy = 1.0
delta_huber_standard = 1.0
delta_huber_scaled = 20.0

# Calculate Influence
psi_cauchy = cauchy_influence(s, c=c_cauchy)
psi_huber_scaled = huber_influence(s, delta=delta_huber_scaled)
psi_huber_standard = huber_influence(s, delta=delta_huber_standard) # For comparison if needed

# Plotting
plt.figure(figsize=(8, 5))
plt.plot(s, psi_cauchy, label=f'Standard Cauchy (c={c_cauchy})', color='red', linewidth=2, linestyle='-')
plt.plot(s, psi_huber_scaled, label=f'Scaled Huber (Ours, $\delta$={delta_huber_scaled})', color='blue', linewidth=2)

# Annotations for Drift Lockout
plt.axvspan(10, 30, color='red', alpha=0.1, label='Drift Lockout Region (>10m)')
plt.axvspan(-30, -10, color='red', alpha=0.1)

plt.text(15, 0.5, 'Zero Gradient\n(Data Rejected)', color='red', fontsize=10, ha='center')
plt.text(25, 18, 'Constant Gradient\n(Recovery Force)', color='blue', fontsize=10, ha='center')

plt.title('Influence Function $\psi(s)$: Cauchy vs. Scaled Huber', fontsize=12)
plt.xlabel('Residual Error $s$ (meters)', fontsize=11)
plt.ylabel('Influence / Gradient $\psi(s)$', fontsize=11)
plt.grid(True, linestyle='--', alpha=0.6)
plt.legend(loc='upper left', fontsize=10)
plt.tight_layout()

# Save
plt.savefig('influence_comparison.pdf')
plt.savefig('influence_comparison.png', dpi=300)
print("Figures saved as influence_comparison.pdf and influence_comparison.png")
