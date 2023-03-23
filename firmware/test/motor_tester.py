#!/usr/bin/env python3
import binascii
from panda import Panda  # install https://github.com/commaai/panda
import argparse
import time
import _thread


# from Msg.h which is from ocelot_controls.dbc
MSG_STEERING_COMMAND_FRAME_ID = 0x22e
MSG_STEERING_STATUS_FRAME_ID = 0x22f
motor_bus_speed = 500  #StepperServoCAN baudrate 500kbps

def motor_tester(motor_bus_num):
  p = Panda()

  p.set_can_speed_kbps(motor_bus_num, motor_bus_speed)

  # Now set the panda from its default of SAFETY_SILENT (read only) to SAFETY_ALLOUTPUT
  print("Setting Panda to output mode...")
  p.set_safety_mode(Panda.SAFETY_ALLOUTPUT)
  p.set_power_save(False) #enable all the busses
  
  _thread.start_new_thread(heartbeat_thread, (p,))


  print("Toggle disable and enable motor control mode - in case it was in Soft Off fault mode")
  p.can_send(MSG_STEERING_COMMAND_FRAME_ID, b"\x30\x00\x00\x00\x00\x00\x00\x00", motor_bus_num)
  p.can_send(MSG_STEERING_COMMAND_FRAME_ID, b"\x40\x10\x00\x00\x00\x00\x00\x00", bus=motor_bus_num)
  
  t_send_prev =0
  t_recv_prev =0
  while True:
    t = time.time()
    if (t - t_send_prev) > 0.01: #send every 10ms in order not to trigger the stop fault
      p.can_send(MSG_STEERING_COMMAND_FRAME_ID, b"\x48\x10\x00\x00\x08\x00\x00\x00", motor_bus_num)
      t_send_prev = t

    can_recv = p.can_recv()
    for address, _, dat, src in can_recv:
      if src == motor_bus_num and address == MSG_STEERING_STATUS_FRAME_ID:
        if t - t_recv_prev > 0.0001:
          hz = 1/(t - t_recv_prev)
        else:
          hz = 0
        print(f"{hz:3.0f}Hz, StatusMsg: {binascii.hexlify(dat)}")
        t_recv_prev = t
    

  #Back to safety...
  print("Disabling output on Panda...")
  p.set_safety_mode(Panda.SAFETY_SILENT)


def heartbeat_thread(p):
  while True:
    try:
      p.send_heartbeat()
      time.sleep(1)
    except:
      break
      raise


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="simple CAN motor test",
                                   formatter_class=argparse.ArgumentDefaultsHelpFormatter)

  parser.add_argument("--bus", type=int, help="CAN bus to which motor is connected", default=2)

  args = parser.parse_args()
  motor_tester(args.bus)