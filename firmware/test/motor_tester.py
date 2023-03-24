#!/usr/bin/env python3
import binascii
from panda import Panda  # install https://github.com/commaai/panda
import argparse
import time
import _thread
import subprocess

def heartbeat_thread(p):
  while True:
    try:
      p.send_heartbeat()
      time.sleep(1)
    except:
      break
      raise

# from Msg.h which is from ocelot_controls.dbc
MSG_STEERING_COMMAND_FRAME_ID = 0x22e
MSG_STEERING_STATUS_FRAME_ID = 0x22f
motor_bus_speed = 500  #StepperServoCAN baudrate 500kbps

#STEERING_COMMAND:STEER_MODE options
OFF = 0
TORQUE_CONTROL = 1
ANGLE_CONTORL = 2
SOFT_OFF = 3


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
                            int(steer_angle * 8),
                            int(steer_torque * 8)
                            )
  checksum = calc_checksum_8bit(packed_data, MSG_STEERING_COMMAND_FRAME_ID)
  packed_data = struct.pack('<B', checksum) + packed_data # add checksum byte at the end
  return packed_data



def motor_tester(motor_bus_num):
  p = Panda()
  p.set_can_speed_kbps(motor_bus_num, motor_bus_speed)
  # Now set the panda from its default of SAFETY_SILENT (read only) to SAFETY_ALLOUTPUT
  print("Setting Panda to All Output mode...")
  p.set_safety_mode(Panda.SAFETY_ALLOUTPUT)
  print("Enable all pandas busses...") #in case panda was sleeping
  p.set_power_save(False) #enable all the busses
  print("Enable panda usb heartbeat..") #so it doesn't stop
  _thread.start_new_thread(heartbeat_thread, (p,))


  #Request off control mode first - in case motor was in SoftOff fault mode
  
  print("\nRequesting motor OFF mode...")
  dat = steering_msg_cmd_data(OFF, 0, 0.0, 0.0)
  print('Sent:' + ''.join('\\x{:02x}'.format(b) for b in dat)) ##b"\x30\x00\x00\x00\x00"
  p.can_send(MSG_STEERING_COMMAND_FRAME_ID, dat, motor_bus_num)

  t_send_prev =0
  cnt_cmd = 0
  t_recv_prev =0
  input("\nRequesting motor TORQUE_CONTROL mode... press a key to start, Ctrl+C to stop")
  while True:
    t = time.time()
    if (t - t_send_prev) > 0.01: #send every 10ms in order not to trigger the stop fault
      dat = steering_msg_cmd_data(cnt_cmd % 0xF, TORQUE_CONTROL, 1.0, 0.0)
      cnt_cmd += 1
      p.can_send(MSG_STEERING_COMMAND_FRAME_ID, dat, motor_bus_num)

      t_send_prev = t

    can_recv = p.can_recv()
    for address, _, dat, src in can_recv:
      if src == motor_bus_num and address == MSG_STEERING_STATUS_FRAME_ID:
        if t - t_recv_prev > 0.0001:
          hz = 1/(t - t_recv_prev)
          print(f"{hz:3.0f}Hz, StatusMsg: {binascii.hexlify(dat)}")
        else:
          hz = 0
        t_recv_prev = t

  #Back to safety...
  print("Disabling output on Panda...")
  p.set_safety_mode(Panda.SAFETY_SILENT)


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Simple motor tester-runner",
                                   formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument("--bus", type=int, help="CAN bus to which motor is connected", default=2)
  args = parser.parse_args()
  print("Killing boardd to release USB...")
  subprocess.run(['pkill', '-f', 'boardd'])
  motor_tester(args.bus)