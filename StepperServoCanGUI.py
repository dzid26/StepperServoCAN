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
MAX_TORQUE = 15.875
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
    # Check for available socketCAN channels
    try:
        channels = []
        output = subprocess.check_output("ip -details -brief link | grep can", shell=True)
        for line in output.decode().split("\n"):
            if line.strip():
                words = line.split()
                if words[1] == "UP" or words[1] == "UNKNOWN":  # Check if channel is up
                    channel = words[0]
                    channels.append(channel)
    except subprocess.CalledProcessError:
        print("No socketCAN channels found.")
        print("Hint: if you want to test the program set up a virtual CAN channel.")
        print("sudo modprobe vcan")
        print("sudo ip link add dev vcan0 type vcan")
        print("sudo ip link set up vcan0")
        return None

    # If multiple CAN channels are found, list them and ask in the terminal which channel to choose
    if len(channels) > 1:
        print("Multiple CAN channels found:")
        for i, channel in enumerate(channels):
            print(f"{i+1}: {channel}")
        print(f"{len(channels)+1}: Abort the program")
        selection = input("Select a channel number: ")
        try:
            selection = int(selection)
            if selection < 1 or selection > len(channels) + 1:
                raise ValueError
            elif selection == len(channels) + 1:
                print("Aborting program...")
                return None
        except ValueError:
            print("Invalid selection")
            return None
        channel = channels[selection - 1]
    # If only 1 channel is found, choose that
    elif len(channels) == 1:
        channel = channels[0]
    # If no channels are found, abort the program
    else:
        print("No socketCAN channels found that are UP.")
        print("Hint: if you want to test the program set up a virtual CAN channel.")
        print("sudo modprobe vcan")
        print("sudo ip link add dev vcan0 type vcan")
        print("sudo ip link set up vcan0")
        return None
    return channel

# Connect to CAN interface that has been selected from GUI and start sending CAN msgs
def connect_to_can_interface(interface):
    global can_enabled
    can_enabled = True
    can_bus = None
    interface_name = interface.get()
    channel = None
    print(f"\nConnecting to {interface_name}...")
    if interface_name == "socketcan":
        if not check_linux():
            can_enabled = False
            return
        print("Finding socketCAN channel...")
        channel = configure_socketcan()
        if channel is None:
            # If send_message thread is already running, stop the while loop by setting can_enable flag to False
            print(f'CAN interface "{interface_name}" not available.')
            can_enabled = False
            # Handle the case where there are no compatible CAN interfaces
            # You can display an error message or perform other actions as needed
            return
    try:
        can_bus = can.Bus(interface=interface_name, bitrate=500000, channel=channel)
        print("Connected.")
    except Exception as e:
        print("An error occurred:", e)
        can_enabled = False

    if can_enabled:
        thread = threading.Thread(target=lambda: send_message(can_bus))
        thread.start()

def interface_selected(event):
    # Run the connect_to_can_interface function when a new option is selected
    interface = selected_backend
    connect_to_can_interface(interface)

def validate_input(input_str, min_value, max_value):
    try:
        # Convert the input string to an floating point number
        value = float(input_str)
        # Check if the value is within the specified range
        if min_value <= value <= max_value:
            return value
    except ValueError:
        # If there was an error converting to an float, or the value is outside the range,
        # catch the exception and do nothing
        pass
    # If we get here, the input was invalid, so return None
    return None

# Add a function to update the message ID
def update_msg_id(event=None):
    global message_id

    # Get the message ID from the widget and convert to integer
    message_id_str = msg_id_var.get()
    if message_id_str.startswith("0x") or message_id_str.startswith("0X"):
        message_id = int(message_id_str, 16)  # Convert from hex
    else:
        message_id = int(message_id_str)  # Convert from decimal

# Function to update torque and angle from widget values
def update_values():
    update_msg_id()

    # Use global variables to update torque and angle values
    global torque, angle

    # Get the torque input string from the widget
    torque_str = torque_widget.get()

    # Check if the input string represents a valid integer within the torque range
    torque = validate_input(torque_str, MIN_TORQUE, MAX_TORQUE)

    # If the input is invalid, set the torque to zero and show an error message
    if torque is None:
        torque = 0
        messagebox.showerror("Error", "Torque value should be between {} and {}".format(MIN_TORQUE, MAX_TORQUE))

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
    global msg, data, counter, torque, angle, message_id

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
    checksum = msg_calc_checksum_8bit(data, lent, message_id)  # Use configurable message ID

    # Encode the data field with new checksum
    data = msg.encode({
        'STEER_TORQUE': torque,
        'STEER_ANGLE': angle,
        'STEER_MODE': steer_mode_widget.get_value(),
        'COUNTER': counter & 0xF,
        'CHECKSUM': checksum
    })

def send_message(can_bus: can.bus.BusABC):
    global can_enabled, message_id

    last_exec_time = time.monotonic()  # current time in seconds since some arbitrary reference point
    loop_count = 0
    last_print_time = time.monotonic()
    while can_enabled:
        # Create a message using the "torque" dbc object
        message = can.Message(arbitration_id=message_id, data=data, is_extended_id=False)
 
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
        
    can_bus.shutdown()

# Define a class named SteerModeWidget
class SteerModeWidget:
    def __init__(self, master, label_text, options_list, command):
        # Create an instance variable of type tk.IntVar to store the selected value
        self.var = tk.IntVar()
        
        # Create a Label widget with the specified label text and place it in the parent widget using the grid geometry manager
        self.label = tk.Label(master, text=label_text)
        self.label.grid(row=3, column=0, sticky="w")  # set sticky to "w" for left alignment
        
        # Create a set of radio buttons, one for each option in the options list
        self.buttons = []
        for idx, option in enumerate(options_list):
            # Create a Radiobutton widget with the specified text and value, and associate it with the var instance variable
            button = tk.Radiobutton(
                master, text=option[1], variable=self.var, value=option[0], command=command
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
torque = 0
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
window.geometry("360x250")  # Set window width and height

# Create labels for the widgets
selected_backend = tk.StringVar(window)
selected_backend.set('pcan')

# Add a new widget for the Message ID
msg_id_var = tk.StringVar(value=hex(msg.frame_id))  # Set default value from msg.frame_id in hex
msg_id_widget = tk.Entry(window, textvariable=msg_id_var)
update_msg_id()
msg_id_widget.bind('<Return>', update_msg_id)  # Bind Enter key
msg_id_widget.bind('<FocusOut>', update_msg_id)  # Bind focus out event

can_interface_selector = ttk.Combobox(window, values=[*sorted(can.interfaces.VALID_INTERFACES)], textvariable=selected_backend)
can_interface_selector.bind("<<ComboboxSelected>>", interface_selected)

can_interface_label = tk.Label(window, text="CAN interface:       ")
msg_id_label = tk.Label(window, text="CAN ID:       ")
torque_label = tk.Label(window, text="Steer Torque:       ")
angle_label = tk.Label(window, text="Steer Angle:       ")

# Set the initial values for torque and angle
initial_torque = torque
initial_angle = angle

# Create the torque and angle widget with an initial value
torque_var = tk.DoubleVar(value=initial_torque)
torque_widget = tk.Entry(window, textvariable=torque_var)
angle_widget = tk.Entry(window, textvariable=tk.StringVar(value=str(initial_angle)))

# Add SteerModeWidget for the Steer Mode option
STEER_MODE_OPTIONS = [
    (0, "Off - instant 0 torque"),
    (1, "TorqueControl"),
    (2, "AngleControl"),
    (3, "SoftOff - ramp torque to 0 at constant rate")
]

steer_mode_widget = SteerModeWidget(window, "Steer Mode:  ", STEER_MODE_OPTIONS, command=update_values)

# Add a button to update torque/angle values
update_button = tk.Button(window, text='Update Torque/Angle value', command=update_values)

# Place the labels and widgets using grid
can_interface_label.grid(row=0, column=0, sticky="w")  # set sticky to "w" for left alignment
can_interface_selector.grid(row=0, column=1, sticky="w")
msg_id_label.grid(row=1, column=0, sticky="w")  # set sticky to "w" for left alignment
msg_id_widget.grid(row=1, column=1, sticky="w")
torque_label.grid(row=2, column=0, sticky="w")  # set sticky to "w" for left alignment
torque_widget.grid(row=2, column=1, sticky="w")
angle_label.grid(row=3, column=0, sticky="w")  # set sticky to "w" for left alignment
angle_widget.grid(row=3, column=1, sticky="w")
update_button.grid(row=9, column=0, columnspan=2, pady=(10, 0))

# Function for closing the program elegantly
def on_closing():
    global can_enabled
    can_enabled = False
    window.destroy()

# Add the quit button
quit_button = tk.Button(window, text="Quit", command=on_closing)
quit_button.grid(row=10, column=0, columnspan=2, pady=(10, 0), sticky=tk.N+tk.S+tk.E+tk.W)

# Trigger events when Esc and Return key are pressed
window.bind('<Escape>', lambda event: on_closing())
window.bind('<Return>', lambda event: update_values())

# This is for making successful program termination when GUI closed from top right corner x button
window.protocol("WM_DELETE_WINDOW", on_closing)

# On startup try to connect to default CAN interface
connect_to_can_interface(selected_backend)

# Run the GUI loop
window.mainloop()
