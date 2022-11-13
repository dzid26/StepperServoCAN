# StepperServoCAN
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import os
import struct
from array import array
from functools import partial
import matplotlib.pyplot as plt
import numpy as np
from scipy import signal
from scipy import optimize

## Firmware stores sensor calibration in a following structure with fields written one after another
# typedef struct {
# 	uint16_t FlashCalData[CALIBRATION_TABLE_SIZE];
# 	uint16_t status;
# 	uint16_t MIN;
# 	uint16_t MAX;
# } FlashCalData_t;
CALIBRATION_STEPS = 32768

class CalibrationRead(object):
    def __init__(self):
        self.cal_size = 400
        self._update_cal_table_size()
        self.address = 0x08007C00  #FLASH_PAGE31_ADDR

        self.struct = self._create_struct_format(self.cal_size) 
        self.values =np.array([])
        self.status= []
        self.MIN = []
        self.MAX = []

        self.wrap_idx = 0

    @staticmethod
    def _create_struct_format(_calsize):
        struct = '<' #ARM has little endian
        struct += str(_calsize) + 'H'  #cal array
        struct += "HHH" #status, min and max
        return struct

    def _update_cal_table_size(self):
        #find calibration size in c-code (if it was manipulated like in my fork)
        f = open("src\BSP\calibration.h", "r")
        for line in f:
            if "#define"  in line and "CALIBRATION_TABLE_SIZE" in line:
                self.cal_size = int(line.split("(",1)[1].split(")",1)[0])  #parse cal size value between brackets
                return
        print("Didn't find cal table size in c-code. The output might be wrong")

    def dump_eeprom_to_file(self):
        #dump eeprom memory for calibration address 
        os.system('ST-LINK_CLI  -NoPrompt -Dump ' + hex(self.address) + ' ' + str(struct.calcsize(self.struct))  + ' eepromCals.bin')


    def load_from_bin(self):
        with open('eepromCals.bin', mode='rb') as dump: # r -read, b -> binary
            values_raw = struct.unpack(self.struct, dump.read(struct.calcsize(self.struct)))
        self.values = np.array(values_raw[0:self.cal_size])
        self.status = values_raw[-3]
        self.MIN = values_raw[-2]
        self.MAX = values_raw[-1]
        self.wrap_idx = self.values.argmin()

    def print_cals(self):
        print("400 measured values:")
        print(self.values)
        print("Status: {0:1}, Min: {1:1}, Max :{2:1}"
            .format(
                self.status,
                self.MIN,
                self.MAX
            )
        )
    
    def fit_func(self, X,  a, b, c, d=0, e=0, f=0, g=0, h=0, i=0, j=0, o=0):
        A =              np.matrix([a, c, e, g, i])
        B = (np.matrix([b, d, f, h, j]))
        K = np.transpose(np.matrix([8, 4, 2, 1, 100]))*3.14/360

        Y = A * np.cos(K*X) + B * np.sin(K*X) + o
        Y = Y.A1 #flatten
        return Y
        
    def plotcals(self):
        x = np.linspace(0, 360 - 1/self.cal_size*360, self.cal_size)  
        y = (self.values - self.values[self.wrap_idx]) / CALIBRATION_STEPS * 360  #normalize the values to start at 0, instead of tiny angle

        y_monot = np.r_[y[self.wrap_idx:], y[0:self.wrap_idx]]
        error = (y_monot-x)
        wrap_point = x[self.wrap_idx]
        start_point_monot = self.cal_size-self.wrap_idx
        
        plt.figure(1)
        ax1 = plt.subplot(2,1,1)
        ax1.plot(x,x, x,y_monot, 'r', x[start_point_monot], y_monot[start_point_monot], '.r', markersize=9, )
        ax1.plot(x,y,'g')
        ax1.legend(("expected angle", "normalized angle", "rotation start point",  "raw angle")); plt.title("Recorded angles")
        
        error_filter2 = signal.wiener(signal.medfilt(error, 5), 5)
        params, params_covariance = optimize.curve_fit(self.fit_func, x, error, p0=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        print(params)

        ax2 = plt.subplot(2,1,2, sharex=ax1)
        ax2.plot(x, error, 'r', x[start_point_monot], error[start_point_monot], '.c',  linewidth=1, markersize=16)
        ax2.plot(x, error_filter2,'k')
        ax2.plot(x, self.fit_func(x, *params),'b');  #fitted complex function
        # ax2.plot(x, error - self.fit_func(x, *params),'g')
        ax2.legend(("calibration data", "rotation start point", "filtered data", "5x cos harmonics fitted", "remaining error after fitting"))
        plt.title("Angle errors")
        ax2.set_xticks(np.linspace(0, 359, 200), minor=True)
        ax2.grid(which='minor', alpha=0.2)

        plt.show()
            
    

if __name__ == "__main__":
    cal = CalibrationRead()

    cal.dump_eeprom_to_file() #currently uses ST-link tool
    cal.load_from_bin()
    cal.print_cals()
    cal.plotcals()

