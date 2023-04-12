# coding: utf8
import numpy as np
import argparse
import math
from time import perf_counter, sleep

import builtins
import keyboard

# Change this with your robot
#from test_bench import TestBench
from solo12 import Solo12

init_done = False
calibration_done = False

def return_callback(event):
    global init_done, calibration_done

    # ignore space keystroke when not asked to
    if not init_done:
        return

    calibration_done = True
    keyboard.unhook_key('space')


def example_script(name_interface):
    # making code compatible with python 2 and 3
    if hasattr(builtins, "raw_input"):
        input = builtins.raw_input
    else:
        input = builtins.input

    global init_done, calibration_done

    # setting up the callback on space keystroke
    keyboard.on_press_key('space', return_callback)

    # Motors not being calibrated will be held at position 0
    q_ref = 0
    v_ref = 0

    kp = 0.125 # Proportional gain
    kd = 0.0025 # Derivative gain

    # Change this with your robot
    device = Solo12(name_interface,dt=0.001)
    nb_motors = device.nb_motors

    print("")
    print("Enter a binary number that represents the motors you want to calibrate.")
    print("LSB: first motor, MSB: last motor")
    print("Example with 8 motors: input \"10011010\" will calibrate motor 1, 3, 4, 7.")
    print("")

    try:
        bin_motors = int(input(), 2); # getting binary representation of motors to be calibrated from user
        print("")
    except:
        print("")
        raise ValueError("Wrong input received, exiting.")

    if bin_motors == 0:
        raise ValueError("0 received, no calibration to be done, exiting.")

    elif bin_motors < 0 or bin_motors >= (1 << nb_motors):
        raise ValueError("An invalid number for your configuration has been received, no calibration to be done, exiting.")

    not_to_calibrate = []
    for i in range(nb_motors):
        if ((bin_motors & (1 << device.motorToUrdf[i])) >> device.motorToUrdf[i]): # if motor needs to be calibrated
            device.encoderOffsets[i] = 0; # set offset to zero
        else:
            not_to_calibrate.append(device.motorToUrdf[i])

    device.Init(calibrateEncoders=True) # finding encoder index happens in here
    init_done = True

    if not device.hardware.IsTimeout():
        print("")
        print("Index detected!")
        print("Please move the motors to calibrate to the desired zero.")
        print("Press the SPACE key when done...")
    
    # --- Control loop ---
    while ((not device.hardware.IsTimeout()) and not calibration_done):
        device.UpdateMeasurment()

        torques = [0]*nb_motors
        for i in not_to_calibrate:
            q_err = q_ref - device.q_mes[i] # Position error
            v_err = v_ref - device.v_mes[i] # Velocity error

            torques[i] = kp * q_err + kd * v_err

        device.SetDesiredJointTorque(torques)
        device.SendCommand(WaitEndOfCycle=True)
    # ---

    if calibration_done:
        device.SetDesiredJointTorque([0] * nb_motors)
        device.SendCommand(WaitEndOfCycle=True)

        print("")
        print("Calibration done.")
        print("Please paste the following value into your RobotHAL file after \"self.encoderOffsets = \":")
        print("")
        print("- np.array({})".format([(device.hardware.GetMotor(i).GetPosition() if (bin_motors & (1 << device.motorToUrdf[i])) >> device.motorToUrdf[i] else - device.encoderOffsets[i]) \
                                        for i in range(nb_motors)]))

    elif device.hardware.IsTimeout():
        print("")
        print("Interface timed out, rerun calibration.")

    print("")
    device.hardware.Stop()  # Shut down the interface between the computer and the master board

def main():
    parser = argparse.ArgumentParser(description='Example masterboard use in python.')
    parser.add_argument('-i',
                        '--interface',
                        required=True,
                        help='Name of the interface (use ifconfig in a terminal), for instance "enp1s0"')

    example_script(parser.parse_args().interface)


if __name__ == "__main__":
    main()
