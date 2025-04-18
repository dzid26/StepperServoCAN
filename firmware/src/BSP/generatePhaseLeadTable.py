# StepperServoCAN

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import numpy as np
import matplotlib.pyplot as plt
import control

# StepperServoCAN uses PWM and low pass filter as a DAC
# It introduces phase shift and needs to be corrected


# Component values
C = 0.1e-6  # Farads
R = 1000    # Ohms
fc = 1 / (2 * np.pi * R * C)  # Cutoff frequency

# Create transfer function (1st order low-pass filter)
sys = control.tf([1], [R * C, 1])

# Frequency parameters
STEPS = 200
Electrical90degINT = 256
maxSpeed = 100  # rev/s
revs_s = np.arange(0, maxSpeed)
f_Hz = revs_s * STEPS / 4  # Convert rev/s to Hz

# Convert frequencies to rad/s for Bode calculation
f_rad = 2 * np.pi * f_Hz

# Calculate specific points for embedded system
mag, phase, omega = control.frequency_response(sys, f_rad)
phase_deg = phase/2/np.pi*360

# Generate Bode plot using control library's built-in plotting
plt.figure()
control.bode(sys, Hz=True, dB=True, deg=True, omega=f_rad)
plt.suptitle('Bode Plot of Low-Pass Filter')
plt.show()

# Generate integer parameters for the firmware
phase_deg_INT = (-phase_deg / 90 * Electrical90degINT).astype(int)
gain_inv_perc_INT = (1 / mag * 16).astype(int)
speedINT_divider = int(maxSpeed * 65536 / len(revs_s))

# Print results
print(f"Cutoff frequency: {fc:.2f} Hz")
print("Frequencies:", f_Hz)
print("Phase INT:", repr(phase_deg_INT))
print("Speed divider:", speedINT_divider)
print("Array length:", len(revs_s))