import numpy as np
import matplotlib.pyplot as plt

# %% setup sine table
SINE_MAX = 32768
SINE_STEPS = 1024
x = np.arange(0, SINE_STEPS) /1024*2*np.pi

y_sin = np.sin(x)
y_cos = np.cos(x)

#alternative commutations
#circular
y_circle_cos=((np.pi/2)**2-(x)**2)**.5 / np.pi*2
y_circle_sin=((np.pi/2)**2-(x-np.pi/2)**2)**.5 / np.pi*2

# triangle
A = 2
P = np.pi
y_line_sin = abs((x-P/2) % (2*P) - P) / P * 2 - 1
y_line_cos = abs(x % (2*P) - P) / P * 2 - 1 

# %% sine profiling - subtract third harmonic
a = .07 # 7%
y_sin_mod = (1-a)*y_sin - a*np.sin(3*x)
y_cos_mod = (1-a)*y_cos + a*np.cos(3*x)

# we want y_sin_mod to be in the range [-1, 1] (in order not to overflow and have good control)
# we also want y_sin_mod to not change direction before crossing zero
# in general both will be satisfied if the gradient of the y_sin_mod is the same as the gradient of the y_sin, we are good
# Their sign will be the same if their product is positive
# d/dx((1-a)sin(x) - a*sin(3x)) * d/dx(sin(x)) > 0
# ((1-a)cos(x) - 3acos(3x)) * cos(x) > 0,  let's use triple-angle trigonometric identity
# ((1-a)cos(x) - 3a(4cos³(x) - 3cos(x)))cos(x) > 0.  let's divide by cos^2(x)
# 1-a -3a(4cos²(x) -3) > 0
# 1 + 8a -12acos²(x) > 0, let's consider two peaks of cos²(x)
# Case 1:  cos²(x)=1
# 1 + 8a - 12a*1 > 0
# -4a > -1
# a < 1/4
#Case 2: cos²(x) = 0
# 1 + 8a > 0
# a > -1/8
# Result:  -1/8 < a < 1/4
#  Since 1/4 is very aggresive, let's keep the bound between -1/8 < a < 1/8

# a4950_comp_sin = np.interp(abs(y_sin*3.3), [0, 1, 2.5, 5],[(8.0 + 10.0)/2, (8.0 + 10.0)/2, (9.0 + 10.0)/2, (9.5+10.5)/2])  / np.interp(3.3, [0, 1, 2.5, 5],[(8.0 + 10.0)/2, (8.0 + 10.0)/2, (9.0 + 10.0)/2, (9.5+10.5)/2]) #accounts for A4950 nonlinearity based on datasheet - it's miniscule though
# a4950_comp_cos = np.interp(abs(y_cos_exp*3.3), [0, 1, 2.5, 5],[(8.0 + 10.0)/2, (8.0 + 10.0)/2, (9.0 + 10.0)/2, (9.5+10.5)/2])  / np.interp(3.3, [0, 1, 2.5, 5],[(8.0 + 10.0)/2, (8.0 + 10.0)/2, (9.0 + 10.0)/2, (9.5+10.5)/2]) #accounts for A4950 nonlinearity


#plot
plt.plot(x, y_sin)
plt.plot(x, y_cos)
plt.plot(x, y_sin_mod)
plt.plot(x, y_cos_mod)
# plt.plot(x, y_circle_sin)
# plt.plot(x, y_circle_cos)
# plt.plot(x, y_line_sin)
# plt.plot(x, y_line_cos)

#phasor plot
plt.plot(x, (y_sin)**2+(y_cos)**2)
plt.plot(x, y_sin_mod**2+y_cos_mod**2)
plt.legend(['sin(x)', 'cos(x)', 'sin(x) mod', 'cos(x) mod', 'magnitude', 'magnitude mod'])
plt.show()

#max phase voltage
plt.plot(x, y_sin + y_cos)
plt.plot(x, y_sin_mod + y_cos_mod)
plt.legend(['combined sin(x) cos(x)', 'combined sin(x)_mod, cos(x)_mod'])
plt.show()
print("Max: " + str(max(y_sin + y_cos)))
print("Max modded: " + str(max(y_sin_mod + y_cos_mod)))

#torque circle
plt.plot((y_sin),(y_cos))
# plt.plot((y_sin_mod*a4950_comp_sin),(y_cos_mod*a4950_comp_cos))
plt.plot(y_sin_mod, y_cos_mod)
# plt.plot(y_line_sin, y_line_cos)
plt.legend(['sin(x)', 'sin(x) mod'])
plt.gca().set_aspect('equal')
plt.annotate('A+', (1, 0), fontsize=12)
plt.annotate('B+', (0, 1), fontsize=12)
plt.annotate('B-', (0, -1), fontsize=12)
plt.annotate('A-', (-1, 0), fontsize=12)
plt.show()


# BEMF rectification when nackdriving the motor
Vfwd = 0.1 #mosfets intrinsic body diode voltage drop at no load
plt.title("BEMF rectification")
plt.plot(x, y_sin)
plt.plot(x, y_cos)
rectified = np.maximum(abs(y_sin)-Vfwd, abs(y_cos)-Vfwd)
plt.plot(x, rectified)
#under no load, it we will see peak stored in a capacitor, not average
rectified_peak = x*0+max(rectified)
plt.plot(x, rectified_peak)

# Assuming perfect sine wave, average can be calculated by first integrating sin(x) between [pi/2,3/4pi]. The result is sqrt(2). 
# Then to get average is sqrt(2)/(3/4pi-pi/2) = 2sqrt(2)/pi. 
# But since we have a voltage drop, we reduce the integral as follows:
# 2*(sqrt(2)-0.15*(pi/2))/pi = 0.75
# rectified_avg = 2*(np.sqrt(2)-Vfwd*(np.pi/2))/np.pi  
# plt.plot(x, x*0+rectified_avg)
plt.legend(['phaseA', 'phaseB','rectified', 'rectified DC peak'])
plt.show



# %% scaling and formating to copy to c code
y_sin_int = (y_sin * (SINE_MAX-1)).round().astype(np.int16)
# y_sin_int_comp = y_sin_int * a4950_comp_sin

#print
with np.printoptions(threshold=SINE_STEPS):
	print(np.array2string(y_sin_int, separator=","))

#Update the C file
import re
with open("sine.c", "r") as file:
    c_file_content = file.read()

# Replace the content between the curly brackets of sineTable
formatted_sine_table = ""
for idx, value in enumerate(y_sin_int):
    formatted_sine_table += f"{value:>6},"
    if (idx + 1) % 16 == 0:
        formatted_sine_table = formatted_sine_table.rstrip(", ") + ",\n"
formatted_sine_table = formatted_sine_table.rstrip(",\n")
c_file_content = re.sub(r"\{(.*?)\}", "{\n" + formatted_sine_table + "\n}", c_file_content, count=1, flags=re.MULTILINE | re.DOTALL)

# Write the updated C file
with open("sine.c", "w") as file:
    file.write(c_file_content)

print("sine.c has been updated with the sine table values.")
