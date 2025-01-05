import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

# Measurements
V = np.array([4.5, 5, 5.5, 6, 6.5, 7, 7.4, 7.5, 8, 8.5, 9, 9.5, 10, 10.5, 11, 11.5, 12, 12.5, 13, 13.5, 14])
RPM = np.array([30, 36, 39.8, 43.3, 45.8, 50.5, 55.2, 55.6, 58.5, 62.5, 66.5, 70.5, 74.5, 76.8, 80, 85, 88.2, 90.5, 96, 99.2, 102.1])

# Additional points
datasheet_values_V = np.array([6, 7.4])
datasheet_values_RPM = np.array([55.6, 62.5])

# Linear regression function
def linear_model(x, a, b):
    return a * x + b

# Fit the initial data
params, covariance = curve_fit(linear_model, V, RPM)

# Parameters of the initial regression
a, b = params
print(f"RPM = {a:.2f} * V + {b:.2f}")

# Generate the fitted curve
V_fit = np.linspace(min(V), max(V), 100)
RPM_fit = linear_model(V_fit, a, b)

# Calculate the linear regression model shift applied to datasheet values
# Compute the average vertical shift between the new points and the initial regression
shifts = datasheet_values_RPM - linear_model(datasheet_values_V, a, b)
average_shift = np.mean(shifts)

# New intercept for the parallel line
parallel_b = b + average_shift
RPM_parallel_fit = linear_model(V_fit, a, parallel_b)

# Plot the data
plt.figure(figsize=(10, 7))
plt.scatter(V, RPM, color="blue", label="Measures")
plt.scatter(datasheet_values_V, datasheet_values_RPM, color="green", label="Datasheet values", marker='x', s=100)
plt.plot(V_fit, RPM_fit, color="red", label=f"Linear regression: RPM = {a:.2f} * V + {b:.2f}")
plt.plot(V_fit, RPM_parallel_fit, color="orange", linestyle="--", label="Linear regression model applied to datasheet values")

# Add the vertical double arrow at the far end (x_max)
x_max = max(V_fit)

y_initial_max = linear_model(x_max, a, b)
y_parallel_max = linear_model(x_max, a, parallel_b)

# Vertical arrow at x_max
plt.annotate(
    "",
    xy=(x_max, y_parallel_max),
    xytext=(x_max, y_initial_max),
    arrowprops=dict(facecolor='black', arrowstyle="<->", lw=2),
    fontsize=10,
    color="black",
)

# Display the average difference next to the arrow
plt.text(
    x_max + 0.1,  # Shift along the X-axis to position the text next to the arrow
    (y_parallel_max + y_initial_max) / 2,  # Position Y in the middle of the arrow
    f"~ {average_shift:.2f} RPM",  # Text with the average difference
    fontsize=10,
    color="black",
)

# Final plot
plt.xlabel("V (Volts)")
plt.ylabel("RPM (revolutions per minute)")
plt.title("LX-16A servo motors RPM vs V curve")
plt.legend()
plt.grid()
plt.show()
