#!/usr/bin/python3

"""
Simple gui program that can be used to test StepperServoCAN motor by @killinen 

"""

import os
import sys
import subprocess

import time
import threading

import tkinter as tk
from tkinter import ttk

from tkinter import messagebox
import cantools
import can

# Constants
MIN_TORQUE = -16
MAX_TORQUE = 16
MIN_ANGLE = -4096
MAX_ANGLE = 4096


##########################################################################
############################### FUNCTIONS ################################
##########################################################################


# This function checks if the operating system is Linux. If not, it prints an error message and aborts the program.
def check_linux():
    """
    Check if the operating system is Linux. If not, print an error message and abort the program.

    Raises:
        SystemExit: If the operating system is not Linux.

    """
    if sys.platform != "linux":
        print("Error: Operating system is not Linux. Aborting.")
        return False
    return True

def msg_calc_checksum_8bit(data: bytes, len: int, msg_id: int) -> int:
    """
    This function calculates 8-bit checksum for given data, length and message id.
    """
    checksum = msg_id
    for i in range(len):
        checksum += data[i]
    checksum = (checksum & 0xFF) + (checksum >> 8)
    checksum &= 0xFF

    return checksum

# This function configures the socketCAN interface in the terminal
def configure_socketcan():

    # Check for available socketCAN interfaces
    interfaces = []
    output = subprocess.check_output("ip -details -brief link | grep can", shell=True)
    for line in output.decode().split("\n"):
        if line.strip():
            words = line.split()
            if words[1] == "UP" or words[1] == "UNKNOWN":  # Check if interface is up
                interface = words[0]
                interfaces.append(interface)
    # If multiple CAN interfaces are found, list them and ask in terminal which interface to choose
    if len(interfaces) > 1:
        print("Multiple CAN interfaces found:")
        for i, interface in enumerate(interfaces):
            print(f"{i+1}: {interface}")
        print(f"{len(interfaces)+1}: Abort the program")
        selection = input("Select an interface number: ")
        try:
            selection = int(selection)
            if selection < 1 or selection > len(interfaces)+1:
                raise ValueError
            elif selection == len(interfaces)+1:
                print("Aborting program...")
                return
        except ValueError:
            print("Invalid selection")
            return
        interface = interfaces[selection-1]
    # If only 1 interface found, just choose that
    elif len(interfaces) == 1:
        interface = interfaces[0]
    # If no interfaces found abort the program
    else:
        print("No CAN interfaces found that are UP, will abort the program.")
        print("Hint: if you want to test the program set up virtual CAN interface with these commands.")
        print("sudo modprobe vcan")
        print("sudo ip link add dev vcan0 type vcan")
        print("sudo ip link set up vcan0")

# Function to search for available CAN interfaces and connect to one.
def connect_to_can_interface(interface):
    """
    This function searches for available CAN interfaces, prompts the user to select one if there are multiple available.
    It then connects to the selected CAN interface using the python-can library and returns a can_bus object.
    """
    interface_name = interface.get()
    if interface_name == "socketcan":
        if not check_linux():
            return
        configure_socketcan()

    # # Connect to selected CAN interface
    # print(f"Connected to CAN interface {interface}")
    can_bus = can.Bus(interface=interface_name, bitrate=500000)

    # Start the send_can_message function in a separate thread
    thread = threading.Thread(target=lambda:send_message(can_bus))
    thread.start()

    return can_bus

def validate_input(input_str, min_value, max_value):
    try:
        # Try to convert the input string to an integer
        value = int(input_str)

        # Check if the integer value is within the specified range
        if min_value <= value <= max_value:
            return value  # If so, return the integer value
    except ValueError:
        # If there was an error converting to an integer, or the value is outside the range,
        # catch the exception and do nothing
        pass

    # If we get here, the input was invalid, so return None
    return None


# Function to update torque and angle from widget values
def update_values():
    # Use global variables to update torque and angle values
    global torque, angle

    # Get the torque input string from the widget
    torque_str = torque_widget.get()

    # Check if the input string represents a valid integer within the torque range
    torque = validate_input(torque_str, MIN_TORQUE, MAX_TORQUE)

    # If the input is invalid, set the torque to zero and show an error message
    if torque is None:
        torque = 0
        messagebox.showerror("Error", "Torque value should be between -16 and 16")

    # Get the angle input string from the widget
    angle_str = angle_widget.get()

    # Check if the input string represents a valid integer within the angle range
    angle = validate_input(angle_str, MIN_ANGLE, MAX_ANGLE)

    # If the input is invalid, set the angle to zero and show an error message
    if angle is None:
        angle = 0
        messagebox.showerror("Error", "Angle value should be between -180 and 180")


# Function to encode and send the CAN message
def update_message():
    global msg, data, counter, torque, angle

    # msg = db.get_message_by_name('STEERING_COMMAND')

    # Calculate new counter value
    counter = counter + 1
    if counter == 16:
        counter = 0

    # Encode data to STEERING_COMMAND data field
    data = msg.encode({
        'STEER_TORQUE': torque,
        'STEER_ANGLE': angle,
        'STEER_MODE': steer_mode_widget.get_value(),
        'COUNTER': counter & 0xF,
        'CHECKSUM': 0
    })

    # Calculate checksum for the STEERING_COMMAND message
    lent = len(data)
    checksum = msg_calc_checksum_8bit(data, lent, 558)

    # Encode the data field with new checksum
    data = msg.encode({
        'STEER_TORQUE': torque,
        'STEER_ANGLE': angle,
        'STEER_MODE': steer_mode_widget.get_value(),
        'COUNTER': counter & 0xF,
        'CHECKSUM': checksum
    })

def send_message(can_bus):
    global can_enabled

    last_exec_time = time.monotonic()  # current time in seconds since some arbitrary reference point
    loop_count = 0
    last_print_time = time.monotonic()
    while can_enabled:
        # Create a message using the "torque" dbc object
        message = can.Message(arbitration_id=msg.frame_id, data=data, is_extended_id=False)
 
        # Update the STEERING_COMMAND message values and send to the bus
        update_message()
        can_bus.send(message)

        # Wait for the remaining time until the next 10 ms interval
        elapsed_time = time.monotonic() - last_exec_time
        remaining_time = max(0.01 - elapsed_time, 0)
        time.sleep(remaining_time)

        # Update last execution time and loop count
        last_exec_time += 0.01
        loop_count += 1

        # Print send frequency every second
        if time.monotonic() - last_print_time >= 1:
            loop_frequency = loop_count / (time.monotonic() - last_print_time)
            print(f"CAN send frequency: {loop_frequency:.2f} Hz")
            loop_count = 0
            last_print_time = time.monotonic()


# Define a class named SteerModeWidget
class SteerModeWidget:
    def __init__(self, master, label_text, options):
        # Create an instance variable of type tk.IntVar to store the selected value
        self.var = tk.IntVar()
        
        # Create a Label widget with the specified label text and place it in the parent widget using the grid geometry manager
        self.label = tk.Label(master, text=label_text)
        self.label.grid(row=3, column=0, sticky="w")  # set sticky to "w" for left alignment
        
        # Create a set of radio buttons, one for each option in the options list
        self.buttons = []
        for idx, option in enumerate(options):
            # Create a Radiobutton widget with the specified text and value, and associate it with the var instance variable
            button = tk.Radiobutton(
                master, text=option[1], variable=self.var, value=option[0]
            )
            
            # Place the radio button in the parent widget using the grid geometry manager, and set sticky to "w" for left alignment
            button.grid(row=3+idx, column=1, sticky="w")
            
            # Add the radio button to the list of buttons
            self.buttons.append(button)

    def get_value(self):
        # Return the selected value as an integer by calling the get method on the var instance variable
        return self.var.get()


##########################################################################
############################ DEFINE VARIABLES ############################
##########################################################################


# Flag to control the CAN traffic
can_enabled = True

# Define Steer Command msg counter value
counter = 0

# Define global variables for torque and angle
torque = 1
angle = 0

# Load the .dbc file and define it's variables
current_dir = os.path.dirname(os.path.abspath(__file__))
dbc_file_path = os.path.join(current_dir, 'opendbc/ocelot_controls.dbc')
db = cantools.database.load_file(dbc_file_path)

msg = db.get_message_by_name('STEERING_COMMAND')

data = msg.encode({
    'STEER_TORQUE': 0,
    'STEER_ANGLE': 0,
    'STEER_MODE': 0,
    'COUNTER': 0,
    'CHECKSUM': 0
})


##########################################################################
############################### MAIN STUFF ###############################
##########################################################################

# Create the GUI window
window = tk.Tk()

# set the title of the window
window.title("StepperServoCAN Tester")

# Set window width and height to custom values
window.geometry("360x260")  # Set window width and height

# Create labels for the widgets
selected_backend = tk.StringVar(window)
selected_backend.set('pcan') #default

can_interface_selector = ttk.OptionMenu(window, selected_backend, selected_backend.get(), *sorted(can.interfaces.VALID_INTERFACES))
can_interface_connect = tk.Button(window, text="Connect", command=lambda:connect_to_can_interface(selected_backend))

can_label = tk.Label(window, text="CAN interface:       ")
torque_label = tk.Label(window, text="Steer Torque:       ")
angle_label = tk.Label(window, text="Steer Angle:       ")


# Set the initial values for torque and angle
initial_torque = 1
initial_angle = 0

# Create the torque and angle widget with an initial value
torque_widget = tk.Entry(window, textvariable=tk.StringVar(value=str(initial_torque)))
angle_widget = tk.Entry(window, textvariable=tk.StringVar(value=str(initial_angle)))

# Add SteerModeWidget for the Steer Mode option
STEER_MODE_OPTIONS = [
    (0, "Off - instant 0 torque"),
    (1, "TorqueControl"),
    (2, "AngleControl"),
    (3, "SoftOff - ramp torque to 0 in 1s")
]

steer_mode_widget = SteerModeWidget(window, "Steer Mode:  ", STEER_MODE_OPTIONS)

# Add a button to update torque/angle values
send_button = tk.Button(window, text='Update Torque/Angle value', command=update_values)

# Place the labels and widgets using grid
can_label.grid(row=0, column=0, sticky="w")  # set sticky to "w" for left alignment
can_interface_selector.grid(row=0, column=1, sticky="w")
can_interface_connect.grid(row=0, column=2, sticky="w")
torque_label.grid(row=1, column=0, sticky="w")  # set sticky to "w" for left alignment
torque_widget.grid(row=1, column=1, sticky="w")
angle_label.grid(row=2, column=0, sticky="w")  # set sticky to "w" for left alignment
angle_widget.grid(row=2, column=1, sticky="w")
send_button.grid(row=8, column=0, columnspan=2)

# Function for closing the program elegantly
def on_closing():
    global can_enabled
    can_enabled = False
    window.destroy()

# Add the quit button
quit_button = tk.Button(window, text="Quit", command=on_closing)
quit_button.grid(row=10, column=0, columnspan=2, pady=10, sticky=tk.N+tk.S+tk.E+tk.W)

window.bind('<Escape>', lambda event: on_closing())
window.bind('<Return>', lambda event: update_values())

# Run the GUI loop
window.mainloop()
