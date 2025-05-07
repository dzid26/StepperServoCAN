#!/usr/bin/env python3

"""
Control motor using Comma's Panda (locally or over ssh). 
(Needs to be flashed with non-release firmare to allow SAFETY_ALLOUTPUT)

For greatest compatibility this script doesn't depend non-built-in modules - 
except for the USB CAN interface (Panda) and it can be run on PC or Comma device. 
Keyboard input values 1,2,3,4... to set torque value
or w,s, keys - control torque dynamically like in a game - supported even over SSH!
s,d step angle in angle mode
m - change control mode 
"""
from panda import Panda  # install https://github.com/commaai/panda

import binascii
import argparse
import time
import _thread
import queue
import threading
import subprocess
import sys

# from Msg.h which is from ocelot_controls.dbc
MSG_STEERING_COMMAND_FRAME_ID = 0x22e
MSG_STEERING_STATUS_FRAME_ID = 0x22f
motor_bus_speed = 500  #StepperServoCAN baudrate 500kbps
MOTOR_MSG_TS = 0.008 # <10Hz
MAX_TORQUE = 9
MAX_ANGLE = 4095

#game mode options
torque_rise_factor = 1.2
torque_decay_factor = 0.8
angle_rise_factor = 1.4
quick_decay_factor = 0.6 #common

#STEERING_COMMAND:STEER_MODE options
modes = {
    'OFF': 0,
    'TORQUE_CONTROL': 1,
    'ANGLE_CONTROL': 2,
    'SOFT_OFF': 3
}

actions = {
    'UP': 1,
    'DOWN': 2,
    'STOP': 3,
    'HOLD': 4
}

def calc_checksum_8bit(work_data, msg_id): # 0xb8 0x1a0 0x19e 0xaa 0xbf
  checksum = msg_id
  for byte in work_data: #checksum is stripped from the data
    checksum += byte     #add up all the bytes

  checksum = (checksum & 0xFF) + (checksum >> 8); #add upper and lower Bytes
  checksum &= 0xFF #throw away anything in upper Byte
  return checksum

import struct
def steering_msg_cmd_data(counter: int, steer_mode: int, steer_torque: float, steer_angle: float) -> bytes:
  # Define the structure format and pack the data
  canbus_fmt = '<Bhb' # msg '<bbhb'  without checksum byte[0]
  packed_data = struct.pack(canbus_fmt,
                            (steer_mode << 4) | counter,
                            max(min(int(steer_angle  * 8), 32767), -32768),
                            max(min(int(steer_torque * 8), 127), -128)
                            )
  checksum = calc_checksum_8bit(packed_data, MSG_STEERING_COMMAND_FRAME_ID)
  packed_data = struct.pack('<B', checksum) + packed_data # add checksum byte at the end
  return packed_data

def rise_and_decay(value:float, delta:float, max_min_limit:float):
  small_value = 0.1
  decay = False
  if delta > 1:
    value += small_value
  elif delta < -1:
    value -= small_value
  else:
    decay = True

  if value*delta > 0 or decay:  #same sign
    value = value * abs(delta)
  else: #if direction change, use quick decay
    value = value * quick_decay_factor 
  
  if value > max_min_limit:
    value = max_min_limit
  elif value < -max_min_limit:
    value = -max_min_limit
  elif value < small_value and value > -small_value: #if value is small, set to 0
    value = 0.0

  return value

def CAN_tx_thread(p:Panda, bus):
  print("Starting CAN TX thread...")
  global _torque
  global _angle
  global _mode
  cnt_cmd = 0
  while True:
    dat = steering_msg_cmd_data(cnt_cmd, _mode, _torque, _angle)
    p.can_send(MSG_STEERING_COMMAND_FRAME_ID, dat, bus)
    cnt_cmd = (cnt_cmd + 1) % 16
    time.sleep(MOTOR_MSG_TS)

def CAN_rx_thread(p, bus):
  t_status_msg_prev =0
  print("Starting CAN RX thread...")
  p.can_clear(bus)     #flush the buffers
  while True:
    time.sleep(MOTOR_MSG_TS/10) #read fast enough so the CAN interface buffer is cleared each loop
    t = time.time()
    can_recv = p.can_recv()
    for address, dat, src in can_recv:
      if src == bus and address == MSG_STEERING_STATUS_FRAME_ID:
        if t - t_status_msg_prev > 0.0001:
          hz = 1/(t - t_status_msg_prev)
        else:
          hz = -1
        clear_last_line()
        print(f"{hz:3.0f}Hz, addr: {address}, bus: {bus}, dat: {binascii.hexlify(dat)}")
        t_status_msg_prev = t



def getChar(): #https://stackoverflow.com/a/36974338/1531161
  # figure out which function to use once, and store it in _func
  if "_func" not in getChar.__dict__:
    try:
      # for Windows-based systems
      import msvcrt # If successful, we are on Windows
      getChar._func=msvcrt.getch
    except ImportError:
      # for POSIX-based systems (with termios & tty support)
      import tty, sys, termios # raises ImportError if unsupported
      def _ttyRead():
        fd = sys.stdin.fileno()
        oldSettings = termios.tcgetattr(fd) # type: ignore
        try:
          tty.setcbreak(fd) # type: ignore
          answer = sys.stdin.read(1)
        finally:
          termios.tcsetattr(fd, termios.TCSADRAIN, oldSettings) # type: ignore
        return answer
      getChar._func=_ttyRead
  return getChar._func()


"""Detects long presses on wsad and perform torque ramping"""
def long_press_value_ramp(key_queue:queue.Queue, break_key:threading.Event):
  global _torque
  while not break_key.is_set():
    time.sleep(0.1) #clear key_queue buffer in 100ms
    try:
      c = key_queue.get_nowait()
    except queue.Empty:
      c = None
    if c == 'w':
      _torque = rise_and_decay(_torque, torque_rise_factor, MAX_TORQUE)
    elif c == 's':
      _torque = rise_and_decay(_torque, -torque_rise_factor, MAX_TORQUE)
    else:
      _torque = rise_and_decay(_torque, torque_decay_factor, MAX_TORQUE)
    print_cmd_state()

def clear_last_line():
  sys.stdout.write("\033[F")  # Move the cursor up one line
  sys.stdout.write("\033[K")  # Clear the line

def print_cmd_state():
  global _torque
  global _angle
  global _mode
  if _mode == modes['TORQUE_CONTROL']:
    clear_last_line()
    print(f"Torque: {_torque:3.2f}\n")
  elif _mode == modes['ANGLE_CONTROL']:
    clear_last_line()
    print(f"Angle:{_angle:4.2f}, FeedForward torque: {_torque:3.2f}\n")

def motor_tester(bus):
  panda = Panda()
  panda.set_heartbeat_disabled()
  panda.set_can_speed_kbps(bus, motor_bus_speed)
  # Now set the panda from its default of SAFETY_SILENT (read only) to SAFETY_ALLOUTPUT
  print("Setting Panda to All Output mode...")
  panda.set_safety_mode(Panda.SAFETY_ALLOUTPUT)
  print("Enable all pandas busses...") #in case panda was sleeping
  panda.set_power_save(False) #enable all the busses

  #Request off control mode first - in case motor
  # or was in SoftOff fault mode
  
  print("\nRequesting motor OFF mode...")
  dat = steering_msg_cmd_data(modes["OFF"], 0, 0.0, 0.0)
  print('Sent:' + ''.join('\\x{:02x}'.format(b) for b in dat)) ##b"\x30\x00\x00\x00\x00"
  panda.can_send(MSG_STEERING_COMMAND_FRAME_ID, dat, bus)
  
  global _torque
  global _angle
  global _mode
  _torque = 0.0
  _angle = 0.0

  key_queue = queue.Queue(maxsize=2) #key buffer
  tx_t = threading.Thread(target=CAN_tx_thread, args=(panda, bus), daemon=True)
  rx_t = threading.Thread(target=CAN_rx_thread, args=(panda, bus), daemon=True)

  _mode = modes['OFF']

  first_run = True
  
  break_long_key = threading.Event()
  long_key_t = None #thread placeholder
  while True:
    if first_run:
      first_run = False
      tx_t.start()
      rx_t.start()
      print(f"Mode: {[name for name, val in modes.items() if val == _mode][0]}\n")
      time.sleep(0.1)
      _mode = modes['TORQUE_CONTROL']
      print(f"Mode: {[name for name, val in modes.items() if val == _mode][0]}\n")
      # rx_t.start()
      print("Enter torque value or used W/S keys to increase/decrease torque and A/D angle. Q to quit.\n\n") #show this before CAN messages spam the terminal
      
    try:
      c = getChar()
      if c == 'q' or c == '\x03':  # Ctrl+C
        break
      if c in {'w', 's'}: #in wsad gammode torque is controlled by keyboard and ramp generator
        if long_key_t is None:
          long_key_t = threading.Thread(target=long_press_value_ramp, args=(key_queue, break_long_key), daemon=True)
          long_key_t.start()
          # because getch is blocking and and because key presses arrive sometimes erratic (SSH)
        try:
          key_queue.put_nowait(c)
        except queue.Full:
          pass #fine
      elif c == 'm': #mode input mode
        _mode = (_mode + 1)%len(modes) # cycle thru modes
        clear_last_line()
        print(f"\nMode: {[name for name, val in modes.items() if val == _mode][0]} ({_mode})\n")
        print_cmd_state()
      elif c == 'd' or c == 'a': #angle input mode
        if _mode == modes['ANGLE_CONTROL']:
          if c == 'd':
            _angle = rise_and_decay(_angle, angle_rise_factor, MAX_ANGLE)
            _torque = max(abs(_torque), 0.1) #match torque signal to angle
          else:
            _angle = rise_and_decay(_angle, -angle_rise_factor, MAX_ANGLE)
            _torque = min(-abs(_torque), -0.1) #match torque signal to angle
          if _angle == 0.0:
            _torque = 0.0
          print_cmd_state()

      elif c in {'1', '2', '3', '4', '5', '6', '7', '8', '9', '0', '-'}: #numeric input mode
        if c == '-':
          c = getChar()
          temp = -float(c)
        else:
          temp = float(c)
        break_long_key.set() #stop long key detection and value ramping
        try:
          if long_key_t is not None:
            long_key_t.join() #wait for thread to finish
            long_key_t = None
        except RuntimeError:
          pass
        break_long_key.clear()  
        _torque = temp
        print_cmd_state()
    except KeyboardInterrupt:
      break

  print("Disabling output on Panda...")
  panda.set_safety_mode(Panda.SAFETY_SILENT)


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Simple motor tester-runner with numeric or game lile controls",
                                   formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument("--bus", type=int, help="CAN bus id to which motor is connected", default=2)
  args = parser.parse_args()
  print("Killing pandad to release USB...")
  try: # useful if run on Comma device
    subprocess.run(['pkill', '-9', '-f', 'pandad'])
    subprocess.run(['/path/to/pandad', '--args-if-any'])
  except:
    pass
  motor_tester(args.bus)
