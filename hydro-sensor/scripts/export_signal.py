#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
    MCC 172 Functions Demonstrated:
        ... (other functions)
"""

# Import ROS related libraries
import rospy
import time
from daqhats import mcc172, OptionFlags, SourceType, HatIDs, HatError
from daqhats_utils import select_hat_device, enum_mask_to_string, chan_list_to_mask
from std_msgs.msg import Float32

# ... (other imports and constants)
voltage_pub = None

def main():
    global voltage_pub
    # ... (other code)
    total_samples_read = 0
    channels = [0]
    channel_mask = chan_list_to_mask(channels)
    num_channels = len(channels)
    samples_read_per_channel = 0
    sensitivity = 1000.0
    samples_per_channel = 10000
    scan_rate = 5120.0
    options = OptionFlags.CONTINUOUS

    try:
        address = select_hat_device(HatIDs.MCC_172)
        hat = mcc172(address)

        for channel in channels:
            hat.iepe_config_write(channel, 0)
            hat.a_in_sensitivity_write(channel, sensitivity)

        hat.a_in_clock_config_write(SourceType.LOCAL, scan_rate)

        synced = False

        while not synced:
            (_source_type, actual_scan_rate, synced) = hat.a_in_clock_config_read()
            if not synced:
                time.sleep(0.005)

        hat.a_in_scan_start(channel_mask, samples_per_channel, options)

        # Initialize ROS node
        rospy.init_node('mcc172_data_publisher', anonymous=True)
        voltage_pub = rospy.Publisher('voltage_data', Float32, queue_size=10)

        # ... (other code)

        # Configure and start the scan.

        read_and_display_data(hat, samples_per_channel, num_channels)
        
        '''
        while True:
            read_result = hat.a_in_scan_read(-1, 0.5)

            if read_result.hardware_overrun:
                print('hardware overrun\n')
                break

            elif read_result.buffer_overrun:
                print('Buffer overrun\n')
                break

            smaples_read_per_channel = int(len(read_result.data) / num_channels)
            total_samples_read += samples_read_per_channel

            if samples_read_per_channel > 0:
                for _i in range(samples_read_per_channel):
                    print("ASD")
                    print(str(read_result.data[_i]))
                    print('\n')
        print(len(ret))
        '''
    except KeyboardInterrupt:
        print("HEllo")
        pass

def read_and_display_data(hat, samples_per_channel, num_channels):
    """
    Reads data from the specified channels on the specified DAQ HAT devices
    and updates the data on the terminal display.  The reads are executed in a
    loop that continues until either the scan completes or an overrun error
    is detected.

    Args:
        hat (mcc172): The mcc172 HAT device object.
        samples_per_channel: The number of samples to read for each channel.
        num_channels (int): The number of channels to display.

    Returns:
        None

    """
    global voltage_pub
    total_samples_read = 0
    read_request_size = 1000
    timeout = 5.0
    r = rospy.Rate(0.001)

    # Since the read_request_size is set to a specific value, a_in_scan_read()
    # will block until that many samples are available or the timeout is
    # exceeded.

    # Continuously update the display value until Ctrl-C is
    # pressed or the number of samples requested has been read.
    while True and not rospy.is_shutdown():
        read_result = hat.a_in_scan_read(-1, 0)

        # Check for an overrun error
        if read_result.hardware_overrun:
            print('\n\nHardware overrun\n')
            break
        elif read_result.buffer_overrun:
            print('\n\nBuffer overrun\n')
            break
        
        samples_read_per_channel = int(len(read_result.data) / num_channels)
        total_samples_read += samples_read_per_channel
        
        #print(samples_read_per_channel)
        if samples_read_per_channel > 0:

            for _i in range(samples_read_per_channel):
                voltage = Float32()
                voltage.data = read_result.data[_i]
                voltage_pub.publish(voltage)
                #print(read_result.data[_i])


    print('good\n')

    return read_result.data

if __name__ == '__main__':
    main()

