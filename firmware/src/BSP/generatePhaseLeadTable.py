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

# StepperServoCAN uses PWM and low pass filter as a DAC
# It introduces phase shift and needs to be corrected

# general first order low pass filter transfer function
def H(w,wc):
    return 1.0 / (1.0 + 1j * (w / wc))



C = 0.1*10**-6   #fahradas
R = 1000        #ohms
fc =  1/ (2*np.pi*R*C) #cut-off frequency for low pass in Hz

STEPS = 200
Electrical90degINT = 256
maxSpeeed = 100 #revs/s
revs_s = np.arange(0, maxSpeeed)
f_Hz = revs_s * STEPS / 4          # full period every 4 steps

phase_deg = -np.arctan(f_Hz / fc) /2/np.pi*360
mag = abs(H(f_Hz, fc))
mag_dB = 20*np.log10(mag)

plt.figure()        # Bode plot
plt.subplot(2,1,1)  
plt.plot(f_Hz,mag_dB); plt.xscale('log')
plt.subplot(2,1,2)
plt.plot(f_Hz,phase_deg); plt.xscale('log')
plt.show()

phase_deg_INT = (-phase_deg / 90 * Electrical90degINT).astype(int)  
gain_inv_perc_INT = (1/mag*16).astype(int)        # 90 electrical deg per step

speedINT_divider = int(maxSpeeed * 65536 / len(revs_s))

print(fc)
print(f_Hz)
print(repr(phase_deg_INT))
# print(gain_inv_perc_INT)
print(speedINT_divider)
print(len(revs_s))


