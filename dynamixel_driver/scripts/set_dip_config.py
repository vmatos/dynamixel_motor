#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2010-2011, Antons Rebguns.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of University of Arizona nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


__author__ = 'Antons Rebguns'
__copyright__ = 'Copyright (c) 2010-2011 Antons Rebguns'

__license__ = 'BSD'
__maintainer__ = 'Antons Rebguns'
__email__ = 'anton@email.arizona.edu'


import sys
from optparse import OptionParser

import roslib
roslib.load_manifest('dynamixel_driver')

from dynamixel_driver import dynamixel_io

if __name__ == '__main__':
    usage_msg = 'Usage: %prog [options] MOTOR_IDs'
    desc_msg = 'Sets various configuration options of specified Dynamixel servo motor.'
    epi_msg = 'Example: %s --port=/dev/ttyUSB1 --baud=57600 --baud-rate=1 --return-delay=1 5 9 23' % sys.argv[0]
    
    parser = OptionParser(usage=usage_msg, description=desc_msg, epilog=epi_msg)
    parser.add_option('-p', '--port', metavar='PORT', default='/dev/ttyUSB0',
                      help='motors of specified controllers are connected to PORT [default: %default]')
    parser.add_option('-b', '--baud', metavar='BAUD', type='int', default=1000000,
                      help='connection to serial port will be established at BAUD bps [default: %default]')
    parser.add_option('-P', '--proportional_gain', type='int', metavar='P_GAIN', dest='proportional_gain',
                      help='set servo motor proportional gain')
    parser.add_option('-D', '--derivative_gain', type='int', metavar='D_GAIN', dest='derivative_gain',
                      help='set servo motor derivative gain')
    parser.add_option('-I', '--integral_gain', type='int', metavar='I_GAIN', dest='integral_gain',
                      help='set servo motor integral gain')
                      
    (options, args) = parser.parse_args(sys.argv)
    print options
    
    if len(args) < 2:
        parser.print_help()
        exit(1)
        
    port = options.port
    baudrate = options.baud
    motor_ids = args[1:]
    
    try:
        dxl_io = dynamixel_io.DynamixelIO(port, baudrate)
    except dynamixel_io.SerialOpenError, soe:
        print 'ERROR:', soe
    else:
        for motor_id in motor_ids:
            motor_id = int(motor_id)
            print 'Configuring Dynamixel motor with ID %d' % motor_id
            if dxl_io.ping(motor_id):
                # check if proportional gain needs to be changed
                if options.proportional_gain is not None:
                    if options.proportional_gain < 0 or options.proportional_gain > 254:
                        print 'Requested proportional gain is out of valid range (0 - 254)'
                    print 'Setting proportional gain to %d' % (options.proportional_gain)
                    dxl_io.set_p_gain(motor_id, options.proportional_gain)
                    
                # check if derivative gain needs to be changed
                if options.derivative_gain is not None:
                    if options.derivative_gain < 0 or options.derivative_gain > 254:
                        print 'Requested derivative gain is out of valid range (0 - 254)'
                    print 'Setting derivative gain to %d' % (options.derivative_gain)
                    dxl_io.set_d_gain(motor_id, options.derivative_gain)
                    
                # check if integral gain needs to be changed
                if options.integral_gain is not None:
                    if options.integral_gain < 0 or options.integral_gain > 254:
                        print 'Requested derivative gain is out of valid range (0 - 254)'
                    print 'Setting integral gain to %d' % (options.integral_gain)
                    dxl_io.set_i_gain(motor_id, options.integral_gain)
                print 'done'
            else:
                print 'Unable to connect to Dynamixel motor with ID %d' % motor_id
