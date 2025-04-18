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

# StepperServoCAN uses PWM and Butterworth low pass filter as a DAC
# The filter introduces phase shift and needs to be corrected

# Component values for 2nd order filter
R1 = 2200      # 2.2k series resistor
C1 = 10e-9     # 10nF to ground
R2 = 100e3     # 100k series resistor
C2 = 220e-12   # 220pF to ground

fc1 = 1 / (2 * np.pi * R1 * C1)
fc2 = 1 / (2 * np.pi * R2 * C2)
fc = fc1 * np.sqrt(2**(1/2)-1)
# print(f"First stage cutoff: {fc1:.2f} Hz")
# print(f"Second stage cutoff: {fc2:.2f} Hz")
print(f"Cutoff frequency: {fc:.2f} Hz")




# Create transfer functions for each stage
sys1 = control.tf([1], [R1*C1, 1])       # First RC stage
sys2 = control.tf([1], [R2*C2, 1])       # Second RC stage
sys_total = sys1 * sys2                  # Combined 2nd order system

# Frequency parameters (unchanged from original)
STEPS = 200
Electrical90degINT = 256
maxSpeed = 100  # rev/s
revs_s = np.arange(0, maxSpeed)
f_Hz = revs_s * STEPS / 4  # Convert rev/s to Hz

# Convert frequencies to rad/s for Bode calculation
f_rad = 2 * np.pi * f_Hz

# Calculate specific points for embedded system
mag, phase, omega = control.frequency_response(sys_total, f_rad)
phase_deg = phase/2/np.pi*360

# Generate Bode plot using control library's built-in plotting
plt.figure()
control.bode(sys_total, Hz=True, dB=True, deg=True, omega=f_rad)
plt.suptitle('Bode Plot of 2nd Order Low-Pass Filter')
plt.show()

# Generate integer parameters for the firmware
phase_deg_INT = (-phase_deg / 90 * Electrical90degINT).astype(int)
gain_inv_perc_INT = (1 / mag * 16).astype(int)
speedINT_divider = int(maxSpeed * 65536 / len(revs_s))

# Print results
# print("Frequencies:", f_Hz)
print("Phase INT:", repr(phase_deg_INT))
print("Array length:", len(revs_s))