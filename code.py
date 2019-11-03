"""
This example display a CircuitPython console and
print which button that is being pressed if any
"""
import time
import board
import digitalio
import busio
import adafruit_lis3dh

if hasattr(board, 'ACCELEROMETER_SCL'):
    i2c = busio.I2C(board.ACCELEROMETER_SCL, board.ACCELEROMETER_SDA)
    int1 = digitalio.DigitalInOut(board.ACCELEROMETER_INTERRUPT)
    lis3dh = adafruit_lis3dh.LIS3DH_I2C(i2c, address=0x19, int1=int1)
else:
    i2c = busio.I2C(board.SCL, board.SDA)
    int1 = digitalio.DigitalInOut(board.D6)  # Set to correct pin for interrupt!
    lis3dh = adafruit_lis3dh.LIS3DH_I2C(i2c, int1=int1)

# Set range of accelerometer (can be RANGE_2_G, RANGE_4_G, RANGE_8_G or RANGE_16_G).
lis3dh.range = adafruit_lis3dh.RANGE_2_G

X_AXIS = 0
Y_AXIS = 1
Z_AXIS = 2
POSITIVE = 0
NEGATIVE = 1

offs_calib = []
gain_calib =[]
axis_offset = []
axis_gain = []
calibrated = []
axis_list =[X_AXIS,Y_AXIS,Z_AXIS]
sign_list = [POSITIVE,NEGATIVE]
menu_dict = {}

# calib_values =[[0 for a1 in range(3)] for a2 in range(3)]
def reset_data():
    global offs_calib
    global gain_calib
    global axis_offset
    global axis_gain
    global calibrated
    offs_calib = [[[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]],
                  [[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]]]
    gain_calib = [[0.0,0.0,0.0],[0.0,0.0,0.0]]
    calibrated = [[False,False,False],[False,False,False]]
    axis_offset = [0.0,0.0,0.0]
    axis_gain = [1.0,1.0,1.0]


def all_calibrated():
    result = True
    for sign in sign_list:
        for axis in axis_list:
            if calibrated[sign][axis] == False:
                result = False
    return result

def list_menu():
    for key in menu_dict:
        print(key, menu_dict[key][0])
def quit():
    do_continue = False

def calibrate_xyz():
    acc_xyz = [3]
    acc_xyz = [value / adafruit_lis3dh.STANDARD_GRAVITY for value in lis3dh.acceleration]
    up_axis = 4
    sign = POSITIVE
    for axis in axis_list:
        #print(acc_xyz[axis])
        if abs(acc_xyz[axis]) > 0.5:
            up_axis = axis
            if acc_xyz[axis] < 0:
                sign = NEGATIVE
            gain_calib[sign][axis]=acc_xyz[axis]
            offs_calib[sign][axis][axis]=0
            calibrated[sign][axis] = True
        else:
            offs_calib[sign][axis][axis]=acc_xyz[axis]

    print(gain_calib)
    print(offs_calib)
    print(calibrated)

    if all_calibrated():
        print('All axis calibrated')
        for axis in axis_list:
            a_off = 0.0
            for sign in sign_list:
                for axis2 in axis_list:
                    a_off = a_off + offs_calib[sign][axis][axis2]
                axis_offset[axis]= a_off / 2.0
        for axis in axis_list:
            axis_gain[axis] = (abs(gain_calib[POSITIVE][axis]+axis_offset[axis])+abs(gain_calib[NEGATIVE][axis]+axis_offset[axis]))/2.0
        print(axis_offset)
        print(axis_gain)


menu_dict = {
    'menu': ['Show commands',list_menu],
    'c': ['Calibrate',calibrate_xyz],
    'q': ['Quit',quit],
    'reset':['Reset data', reset_data]}

def menu_functions(inp):
    if inp in menu_dict:
        print(menu_dict[inp][0])
        menu_dict[inp][1]()

def print_values():
    acc_values=  [value / adafruit_lis3dh.STANDARD_GRAVITY for value in lis3dh.acceleration]
    print("Raw: x = {0:3f} G, y = {1:3f} G, z = {2:3f} G".format(acc_values[X_AXIS], acc_values[Y_AXIS], acc_values[Z_AXIS]))
    if all_calibrated():
        acc_values_fixed = [(acc_values[axis]-axis_offset[axis] ) / axis_gain[axis] for axis in axis_list]
        print("Fix: x = {0:3f} G, y = {1:3f} G, z = {2:3f} G".format(acc_values_fixed[X_AXIS], acc_values_fixed[Y_AXIS], acc_values_fixed[Z_AXIS]))


do_continue = True
print('XYZ calibration')
reset_data()

while do_continue:
    s = input().lower()
    if len(s) == 0:
        print_values()
    else:
        menu_functions(s)

