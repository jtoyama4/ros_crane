#!/usr/bin/env python
# coding: utf-8

import sys
import roslib
roslib.load_manifest('dynamixel_driver')

from dynamixel_driver import dynamixel_io


if __name__ == '__main__':
    port = '/dev/ttyUSB0'
    baudrate = 1000000
    motor_ids = [2,3,4,5,6,7,8,9,10]
    angles = [[537, 3559], [976, 3100], [516, 2500], [445, 2097], [1887, 4060], [3716, 2460], [893, 3189], [885, 3202], [1230, 2045]]
    try:
        dxl_io = dynamixel_io.DynamixelIO(port, baudrate)
    except dynamixel_io.SerialOpenError, soe:
        print 'ERROR', soe
    else:
        for motor_id, angle in zip(motor_ids, angles):
            n = 0
            print 'Configuring Dynamixel motor with ID %d' % motor_id
            while True:
                try:
                    dxl_io.set_angle_limit_cw(motor_id, angle[0])
                    
                    dxl_io.set_angle_limit_ccw(motor_id, angle[1])

                    dxl_io.set_speed(motor_id, 64)
                    
                    print 'Setting CW angle limit to %d' % angle[0]
                    print 'Setting CCW angle limit to %d' % angle[1]
                    print 'Setting Speed limit 64'
                    break
                except:
                    n += 1
                    if n == 50:
                        "something wrong with %d!!!" % motor_id
                        break
                    pass
            
            
