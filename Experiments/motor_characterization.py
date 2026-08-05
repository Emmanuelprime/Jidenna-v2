import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy import stats
from scipy.optimize import curve_fit

# Read data
df = pd.read_csv('motor_data.csv')

print("Columns in CSV:", df.columns.tolist())
print("\nFirst few rows:")
print(df.head())

def linear_model(x, a, b):
    return a * x + b

def quadratic_model(x, a, b, c):
    return a * x**2 + b * x + c

# Average velocities for each PWM combination - FIXED
df_avg = df.groupby(['left_pwm', 'right_pwm']).agg({
    'vl': ['mean', 'std'],
    'vr': ['mean', 'std']
}).reset_index()

# Flatten multi-index columns
df_avg.columns = ['left_pwm', 'right_pwm', 'vl_mean', 'vl_std', 'vr_mean', 'vr_std']

print("\nAveraged data:")
print(df_avg.head())

# Separate left and right data
left_data = df_avg[['left_pwm', 'vl_mean', 'vl_std']].copy()
left_data.columns = ['pwm', 'velocity', 'std']
left_data['wheel'] = 'Left'

right_data = df_avg[['right_pwm', 'vr_mean', 'vr_std']].copy()
right_data.columns = ['pwm', 'velocity', 'std']
right_data['wheel'] = 'Right'

all_data = pd.concat([left_data, right_data])

print("\n=== PWM to Velocity Analysis ===\n")

# Store results for each wheel
results = {}

for wheel in ['Left', 'Right']:
    print(f"\n{'-'*40}")
    print(f"{wheel} Wheel")
    print(f"{'-'*40}")
    
    wheel_data = all_data[all_data['wheel'] == wheel]
    pwm = wheel_data['pwm'].values
    vel = wheel_data['velocity'].values
    
    # Remove any NaN or zero values if they exist
    mask = (vel > 0) & (~np.isnan(vel))
    pwm = pwm[mask]
    vel = vel[mask]
    
    if len(pwm) < 2:
        print("Not enough data points for fitting")
        continue
    
    # Linear fit
    slope, intercept, r_value, p_value, std_err = stats.linregress(pwm, vel)
    results[wheel] = {'slope': slope, 'intercept': intercept, 'r2': r_value**2}
    
    print(f"\nLinear Model: v = {slope:.4f}*PWM + {intercept:.4f}")
    print(f"R² = {r_value**2:.4f}")
    print(f"Standard Error: {std_err:.4f}")
    
    # Quadratic fit
    try:
        popt, pcov = curve_fit(quadratic_model, pwm, vel)
        a, b, c = popt
        vel_pred = quadratic_model(pwm, a, b, c)
        residuals = vel - vel_pred
        ss_res = np.sum(residuals**2)
        ss_tot = np.sum((vel - np.mean(vel))**2)
        r2_quad = 1 - (ss_res / ss_tot)
        
        print(f"\nQuadratic Model: v = {a:.4f}*PWM² + {b:.4f}*PWM + {c:.4f}")
        print(f"R² = {r2_quad:.4f}")
    except:
        print("\nQuadratic fit failed (not enough data points)")
    
    print(f"\nConversion factor: {slope:.4f} m/s per PWM unit")
    print(f"At PWM=80: {slope*80 + intercept:.4f} m/s")

print(f"\n{'-'*40}")
print("Summary Statistics")
print(f"{'-'*40}")

print(f"\nLeft Wheel:")
print(f"  Min velocity: {left_data['velocity'].min():.4f} m/s")
print(f"  Max velocity: {left_data['velocity'].max():.4f} m/s")
print(f"  Mean velocity: {left_data['velocity'].mean():.4f} m/s")

print(f"\nRight Wheel:")
print(f"  Min velocity: {right_data['velocity'].min():.4f} m/s")
print(f"  Max velocity: {right_data['velocity'].max():.4f} m/s")
print(f"  Mean velocity: {right_data['velocity'].mean():.4f} m/s")

# Check for asymmetry
min_len = min(len(left_data['velocity'].values), len(right_data['velocity'].values))
diff = left_data['velocity'].values[:min_len] - right_data['velocity'].values[:min_len]
print(f"\nLeft-Right difference: {np.mean(diff):.4f} m/s")
print(f"  Standard deviation: {np.std(diff):.4f}")

# Create plots
fig, axes = plt.subplots(2, 2, figsize=(12, 10))

# Left wheel
ax1 = axes[0, 0]
ax1.errorbar(left_data['pwm'], left_data['velocity'], 
             yerr=left_data['std'], fmt='o', capsize=5, 
             label='Data', alpha=0.6)
ax1.set_xlabel('PWM')
ax1.set_ylabel('Velocity (m/s)')
ax1.set_title('Left Wheel: PWM vs Velocity')
ax1.grid(True, alpha=0.3)

pwm_fit = np.linspace(0, 80, 100)
if 'Left' in results:
    slope_l = results['Left']['slope']
    intercept_l = results['Left']['intercept']
    vel_lin_l = linear_model(pwm_fit, slope_l, intercept_l)
    ax1.plot(pwm_fit, vel_lin_l, 'r-', 
             label=f'Linear: v={slope_l:.3f}x+{intercept_l:.3f}')
ax1.legend()

# Right wheel
ax2 = axes[0, 1]
ax2.errorbar(right_data['pwm'], right_data['velocity'], 
             yerr=right_data['std'], fmt='o', capsize=5,
             label='Data', alpha=0.6)
ax2.set_xlabel('PWM')
ax2.set_ylabel('Velocity (m/s)')
ax2.set_title('Right Wheel: PWM vs Velocity')
ax2.grid(True, alpha=0.3)

if 'Right' in results:
    slope_r = results['Right']['slope']
    intercept_r = results['Right']['intercept']
    vel_lin_r = linear_model(pwm_fit, slope_r, intercept_r)
    ax2.plot(pwm_fit, vel_lin_r, 'b-', 
             label=f'Linear: v={slope_r:.3f}x+{intercept_r:.3f}')
ax2.legend()

# Comparison
ax3 = axes[1, 0]
ax3.scatter(left_data['pwm'], left_data['velocity'], label='Left Wheel', alpha=0.6)
ax3.scatter(right_data['pwm'], right_data['velocity'], label='Right Wheel', alpha=0.6)
if 'Left' in results:
    ax3.plot(pwm_fit, vel_lin_l, 'r-', alpha=0.5)
if 'Right' in results:
    ax3.plot(pwm_fit, vel_lin_r, 'b-', alpha=0.5)
ax3.set_xlabel('PWM')
ax3.set_ylabel('Velocity (m/s)')
ax3.set_title('Left vs Right Wheel Comparison')
ax3.grid(True, alpha=0.3)
ax3.legend()

# Residuals
ax4 = axes[1, 1]
if 'Left' in results:
    left_pred = linear_model(left_data['pwm'].values, slope_l, intercept_l)
    residuals_left = left_data['velocity'].values - left_pred
    ax4.scatter(left_data['pwm'], residuals_left, label='Left Residuals', alpha=0.6)
if 'Right' in results:
    right_pred = linear_model(right_data['pwm'].values, slope_r, intercept_r)
    residuals_right = right_data['velocity'].values - right_pred
    ax4.scatter(right_data['pwm'], residuals_right, label='Right Residuals', alpha=0.6)
ax4.axhline(y=0, color='k', linestyle='--', alpha=0.3)
ax4.set_xlabel('PWM')
ax4.set_ylabel('Residuals (m/s)')
ax4.set_title('Linear Fit Residuals')
ax4.grid(True, alpha=0.3)
ax4.legend()

plt.tight_layout()
plt.savefig('pwm_velocity_analysis.png', dpi=300)
plt.show()

# Mapping functions
if 'Left' in results and 'Right' in results:
    print(f"\n{'-'*40}")
    print("PWM to Velocity Mapping Functions")
    print(f"{'-'*40}")

    def left_velocity(pwm):
        return slope_l * pwm + intercept_l

    def right_velocity(pwm):
        return slope_r * pwm + intercept_r

    def pwm_from_velocity_left(velocity):
        return (velocity - intercept_l) / slope_l

    def pwm_from_velocity_right(velocity):
        return (velocity - intercept_r) / slope_r

    print("\nTest mappings:")
    for pwm in [20, 40, 60, 80]:
        vl = left_velocity(pwm)
        vr = right_velocity(pwm)
        print(f"PWM={pwm:2d} → Left: {vl:.4f} m/s, Right: {vr:.4f} m/s")
        print(f"    Diff: {vl-vr:.4f} m/s")

    # Save mapping
    with open('pwm_mapping.txt', 'w') as f:
        f.write("PWM to Velocity Mapping\n")
        f.write("="*50 + "\n\n")
        f.write(f"Left Wheel: v = {slope_l:.6f}*PWM + {intercept_l:.6f}\n")
        f.write(f"  R² = {results['Left']['r2']:.6f}\n\n")
        f.write(f"Right Wheel: v = {slope_r:.6f}*PWM + {intercept_r:.6f}\n")
        f.write(f"  R² = {results['Right']['r2']:.6f}\n\n")
        f.write("PWM\tLeft(m/s)\tRight(m/s)\n")
        for pwm in range(0, 85, 5):
            f.write(f"{pwm}\t{left_velocity(pwm):.6f}\t{right_velocity(pwm):.6f}\n")

    print("\nMapping saved to pwm_mapping.txt")
else:
    print("\nCould not generate mapping - check your data")