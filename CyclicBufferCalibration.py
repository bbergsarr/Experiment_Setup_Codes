import serial
import struct
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

def read_array():
    # Open serial port (adjust 'COM3' to your port)
    ser = serial.Serial('COM3', 115200, timeout=10)
    print("Waiting for data from Arduino...")

    # Wait for start byte (0xAA)
    while True:
        if ser.in_waiting > 0:
            start_byte = ser.read(1)
            if start_byte == b'\xAA':
                break

    # Read array size
    size_bytes = ser.read(1)
    if len(size_bytes) != 1:
        print("Error: Could not read array size")
        ser.close()
        return
    size = struct.unpack('B', size_bytes)[0]
    print(f"Array size: {size}")

    # Read RPM data
    data = []
    for _ in range(size):
        raw = ser.read(4)  # Read 4 bytes for each float
        if len(raw) != 4:
            print(f"Error: Incomplete data at index {_}, received {len(raw)} bytes")
            break
        value = struct.unpack('f', raw)[0]
        data.append(value)

    print("Received array:", data)
    ser.close()

    # Process the received data
    if len(data) == size:
        process_data(data, size)
    else:
        print("Error: Data length mismatch")

def process_data(array, size):
    # Set PWM order (True for increasing: 0 to 255, False for decreasing: 255 to 0)
    increasing_order = True  # Change this to False for decreasing PWM
    
    # Define PWM values based on order
    if increasing_order:
        pwm_values = np.linspace(0, 255, size)  # 0, 5, 10, ..., 255
        title_suffix = "Increasing PWM"
        q = 8  # Threshold index for PWM >= 40
        pwm_fit = pwm_values[q:]  # From PWM 40 upward
        rpm_fit = array[q:]
        pwm_threshold = 40
    else:
        pwm_values = np.linspace(255, 0, size)  # 255, 250, 245, ..., 0
        title_suffix = "Decreasing PWM"
        q = 8  # Threshold index for PWM >= 40 (now near end of array)
        pwm_fit = pwm_values[:size - q]  # From 255 down to PWM 40
        rpm_fit = array[:size - q]
        pwm_threshold = 40

    rpm_values = np.array(array)

    # 1. PWM to RPM Fit
    def fit_func_rpm(pwm, a, b, c, d):
        return a * pwm**4 + b * pwm**3 + c * pwm**2 + d * pwm

    params_rpm, _ = curve_fit(fit_func_rpm, pwm_fit, rpm_fit)
    print(f"Fitted Parameters (PWM to RPM, {title_suffix}):", params_rpm)

    pwm_smooth = np.linspace(0, 255, 500) if increasing_order else np.linspace(255, 0, 500)

    def piecewise_rpm(pwm):
        rpm_fitted = fit_func_rpm(pwm, *params_rpm)
        return np.where(pwm < pwm_threshold, 0, rpm_fitted) if increasing_order else np.where(pwm < pwm_threshold, 0, rpm_fitted)

    rpm_fitted = piecewise_rpm(pwm_smooth)

    plt.figure(figsize=(8, 6))
    plt.plot(pwm_values, rpm_values, 'go', label='Measured RPM')
    plt.plot(pwm_smooth, rpm_fitted, 'm-', label='Fitted Curve')
    plt.title(f'PWM vs RPM Graph ({title_suffix})')
    plt.xlabel('PWM')
    plt.ylabel('RPM')
    plt.legend()
    plt.grid()
    plt.savefig(f'PWMvsRPM_Graph_{title_suffix}.svg', format='svg')
    plt.show()

    # 2. RPM to PWM Fit
    rpm_fit_inv = array[q:] if increasing_order else array[:size - q]
    pwm_fit_inv = pwm_values[q:] if increasing_order else pwm_values[:size - q]

    def fit_func_pwm(rpm, a, b, c, d):
        return a * rpm**4 + b * rpm**3 + c * rpm**2 + d * rpm

    params_pwm, _ = curve_fit(fit_func_pwm, rpm_fit_inv, pwm_fit_inv)
    print(f"Fitted Parameters (RPM to PWM, {title_suffix}):", params_pwm)

    rpm_smooth = np.linspace(0, max(rpm_values), 500)

    def piecewise_pwm(rpm):
        pwm_fitted = fit_func_pwm(rpm, *params_pwm)
        threshold_rpm = rpm_values[q] if increasing_order else rpm_values[size - q]
        return np.where(rpm < threshold_rpm, 0, pwm_fitted)

    pwm_fitted = piecewise_pwm(rpm_smooth)

    plt.figure(figsize=(8, 6))
    plt.plot(rpm_values, pwm_values, 'go', label='Measured PWM')
    plt.plot(rpm_smooth, pwm_fitted, 'm-', label='Fitted Curve')
    plt.title(f'RPM vs PWM Graph ({title_suffix})')
    plt.xlabel('RPM')
    plt.ylabel('PWM')
    plt.legend()
    plt.grid()
    plt.savefig(f'RPMvsPWM_Graph_{title_suffix}.svg', format='svg')
    plt.show()

    # Error Calculations
    def M(rpm):
        return fit_func_pwm(rpm, *params_pwm)

    def N(pwm):
        return fit_func_rpm(pwm, *params_rpm)

    def error_pwm(pwm):
        return M(N(pwm)) - pwm

    def error_rpm(rpm):
        return N(M(rpm)) - rpm

    def piecewise_error_pwm(pwm_smooth):
        error_pwm_values = np.zeros_like(pwm_smooth)
        mask = pwm_smooth > pwm_threshold if increasing_order else pwm_smooth > pwm_threshold
        error_pwm_values[mask] = error_pwm(pwm_smooth[mask])
        return error_pwm_values

    def piecewise_error_rpm(rpm_smooth):
        error_rpm_values = np.zeros_like(rpm_smooth)
        threshold_rpm = rpm_values[q] if increasing_order else rpm_values[size - q]
        mask = rpm_smooth >= threshold_rpm
        error_rpm_values[mask] = error_rpm(rpm_smooth[mask])
        return error_rpm_values

    error_pwm_values = piecewise_error_pwm(pwm_smooth)
    error_rpm_values = piecewise_error_rpm(rpm_smooth)

    # Error Plots
    plt.figure(figsize=(8, 6))
    plt.plot(pwm_smooth, error_pwm_values, 'b-', label='Error in PWM')
    plt.title(f'Error in PWM vs PWM ({title_suffix})')
    plt.xlabel('PWM')
    plt.ylabel('Error in PWM')
    plt.legend()
    plt.grid()
    plt.savefig(f'ErrorInPWM_{title_suffix}.svg', format='svg')
    plt.show()

    plt.figure(figsize=(8, 6))
    plt.plot(rpm_smooth, error_rpm_values, 'r-', label='Error in RPM')
    plt.title(f'Error in RPM vs RPM ({title_suffix})')
    plt.xlabel('RPM')
    plt.ylabel('Error in RPM')
    plt.legend()
    plt.grid()
    plt.savefig(f'ErrorInRPM_{title_suffix}.svg', format='svg')
    plt.show()

if __name__ == "__main__":
    read_array()