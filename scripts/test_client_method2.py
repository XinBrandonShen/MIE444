'''
This file is part of SimMeR, an educational mechatronics robotics simulator.
Initial development funded by the University of Toronto MIE Department.
Copyright (C) 2023  Ian G. Bennett

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as published
by the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
'''

# Basic client for sending and receiving data to SimMeR or a robot, for testing purposes
# Some code modified from examples on https://realpython.com/python-sockets/
# and https://www.geeksforgeeks.org/python-display-text-to-pygame-window/

# If using a bluetooth low-energy module (BT 4.0 or higher) such as the HM-10, the ble-serial
# package (https://github.com/Jakeler/ble-serial) is necessary to directly create a serial
# connection between a computer and the device. If using this package, the BAUDRATE constant
# should be left as the default 9600 bps.

import threading
import socket
import time
from datetime import datetime
import serial


# Wrapper functions
def transmit(data):
    '''Selects whether to use serial or tcp for transmitting.'''
    if SIMULATE:
        transmit_tcp(data)
    else:
        transmit_serial(data)
    time.sleep(TRANSMIT_PAUSE)

def receive():
    '''Selects whether to use serial or tcp for receiving.'''
    if SIMULATE:
        return receive_tcp()
    else:
        return receive_serial()

# TCP communication functions
def transmit_tcp(data):
    '''Send a command over the TCP connection.'''
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.connect((HOST, PORT_TX))
            s.send(data.encode('ascii'))
        except (ConnectionRefusedError, ConnectionResetError):
            print('Tx Connection was refused or reset.')
        except TimeoutError:
            print('Tx socket timed out.')
        except EOFError:
            print('\nKeyboardInterrupt triggered. Closing...')

def receive_tcp():
    '''Receive a reply over the TCP connection.'''
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s2:
        try:
            s2.connect((HOST, PORT_RX))
            response_raw = s2.recv(1024).decode('ascii')
            if response_raw:
                # return the data received as well as the current time
                return [depacketize(response_raw), datetime.now().strftime("%H:%M:%S")]
            else:
                return [[False], datetime.now().strftime("%H:%M:%S")]
        except (ConnectionRefusedError, ConnectionResetError):
            print('Rx connection was refused or reset.')
        except TimeoutError:
            print('Response not received from robot.')

# Serial communication functions
def transmit_serial(data):
    '''Transmit a command over a serial connection.'''
    clear_serial()
    SER.write(data.encode('ascii'))

def receive_serial():
    '''Receive a reply over a serial connection.'''

    start_time = time.time()
    response_raw = ''
    while time.time() < start_time + TIMEOUT_SERIAL:
        if SER.in_waiting:
            response_char = SER.read().decode('ascii')
            if response_char == FRAMEEND:
                response_raw += response_char
                break
            else:
                response_raw += response_char

    print(f'Raw response was: {response_raw}')

    # If response received, return it
    if response_raw:
        return [depacketize(response_raw), datetime.now().strftime("%H:%M:%S")]
    else:
        return [[False], datetime.now().strftime("%H:%M:%S")]

def clear_serial(delay_time: float = 0):
    '''Wait some time (delay_time) and then clear the serial buffer.'''
    if SER.in_waiting:
        time.sleep(delay_time)
        print(f'Clearing Serial... Dumped: {SER.read(SER.in_waiting)}')

# Packetization and validation functions
def depacketize(data_raw: str):
    '''
    Take a raw string received and verify that it's a complete packet, returning just the data messages in a list.
    '''

    # Locate start and end framing characters
    start = data_raw.find(FRAMESTART)
    end = data_raw.find(FRAMEEND)

    # Check that the start and end framing characters are present, then return commands as a list
    if (start >= 0 and end >= start):
        data = data_raw[start+1:end].replace(f'{FRAMEEND}{FRAMESTART}', ',').split(',')
        cmd_list = [item.split(':', 1) for item in data]

        # Make sure this list is formatted in the expected manner
        for cmd_single in cmd_list:
            match len(cmd_single):
                case 0:
                    cmd_single.append('')
                    cmd_single.append('')
                case 1:
                    cmd_single.append('')
                case 2:
                    pass
                case _:
                    cmd_single = cmd_single[0:2]

        return cmd_list
    else:
        return [[False, '']]

def packetize(data: str):
    '''
    Take a message that is to be sent to the command script and packetize it with start and end framing.
    '''

    # Check to make sure that a packet doesn't include any forbidden characters (0x01, 0x02, 0x03, 0x04)
    forbidden = [FRAMESTART, FRAMEEND, '\n']
    check_fail = any(char in data for char in forbidden)

    if not check_fail:
        return FRAMESTART + data + FRAMEEND

    return False

def response_string(cmds: str, responses_list: list):
    '''
    Build a string that shows the responses to the transmitted commands that can be displayed easily.
    '''
    # Validate that the command ids of the responses match those that were sent
    cmd_list = [item.split(':')[0] for item in cmds.split(',')]
    valid = validate_responses(cmd_list, responses_list)

    # Build the response string
    out_string = ''
    sgn = ''
    chk = ''
    for item in zip(cmd_list, responses_list, valid):
        if item[2]:
            sgn = '='
            chk = '✓'
        else:
            sgn = '!='
            chk = 'X'

        out_string = out_string + (f'cmd {item[0]} {sgn} {item[1][0]} {chk}, response "{item[1][1]}"\n')

    return out_string

def validate_responses(cmd_list: list, responses_list: list):
    '''
    Validate that the list of commands and received responses have the same command id's. Takes a
    list of commands and list of responses as inputs, and returns a list of true and false values
    indicating whether each id matches.
    '''
    valid = []
    for pair in zip(cmd_list, responses_list):
        if pair[1]:
            if pair[0] == pair[1][0]:
                valid.append(True)
            else:
                valid.append(False)
    return valid




############## Constant Definitions Begin ##############
### Network Setup ###
HOST = '127.0.0.1'      # The server's hostname or IP address
PORT_TX = 61200         # The port used by the *CLIENT* to receive
PORT_RX = 61201         # The port used by the *CLIENT* to send data

### Serial Setup ###
BAUDRATE = 9600         # Baudrate in bps
PORT_SERIAL = 'COM3'    # COM port identification
TIMEOUT_SERIAL = 1      # Serial port timeout, in seconds

### Packet Framing values ###
FRAMESTART = '['
FRAMEEND = ']'
CMD_DELIMITER = ','

### Set whether to use TCP (SimMeR) or serial (Arduino) ###
SIMULATE = True



############### Initialize ##############
### Source to display
if SIMULATE:
    SOURCE = 'SimMeR'
else:
    SOURCE = 'serial device ' + PORT_SERIAL
try:
    SER = serial.Serial(PORT_SERIAL, BAUDRATE, timeout=TIMEOUT_SERIAL)
except serial.SerialException:
    print(f'Serial connection was refused.\nEnsure {PORT_SERIAL} is the correct port and nothing else is connected to it.')

### Pause time after sending messages
if SIMULATE:
    TRANSMIT_PAUSE = 0.1
else:
    TRANSMIT_PAUSE = 0





ULTRASONIC_THRESHOLD = 2  # Threshold to slow down when approaching obstacle (in inches)
REVERSE_DISTANCE = 2  # Distance to reverse when an obstacle is detected (in inches)
OBSTACLE_AVOIDING = False  # Flag to indicate when obstacle avoidance is active

############## Main section for the communication client ##############
RUN_COMMUNICATION_CLIENT = True  # To control the main loop

def command_execution_loop():
    '''Main loop to receive and execute typed commands'''
    global OBSTACLE_AVOIDING
    while RUN_COMMUNICATION_CLIENT:
        cmd = input('Type in a string to send: ')
        if OBSTACLE_AVOIDING:
            print("Obstacle avoidance is active, pausing typed commands.")
            continue  # Skip user command if avoidance is active

        # Send the command
        packet_tx = packetize(cmd)
        if packet_tx:
            transmit(packet_tx)
        
        # Receive the response
        result = receive()
        if result is not None:
            [responses, time_rx] = result
            if responses[0]:
                print(f"At time '{time_rx}' received from {SOURCE}:\n{response_string(cmd, responses)}")
            else:
                print(f"At time '{time_rx}' received from {SOURCE}:\nMalformed Packet")
        else:
            print("No response received or connection issue occurred.")


def check_ultrasonic_sensors():
    '''Check ultrasonic sensors for potential obstacles and return readings'''
    readings = {}  # Store the readings from all sensors
    for sensor_id in ['u0', 'u1', 'u2', 'u3']:  # Assuming 4 ultrasonic sensors
        packet_tx = packetize(sensor_id)
        if packet_tx:
            transmit(packet_tx)
            result = receive()
            if result is not None:
                [responses, time_rx] = result
                sensor_reading = float(responses[0][1])
                readings[sensor_id] = sensor_reading
                # Debugging
                #print(f"Ultrasonic sensor {sensor_id} reading: {sensor_reading} inches")
                #print(f"Failed to receive a response from sensor {sensor_id}.")
    return readings


def avoid_obstacle(readings):
    '''Determine which direction to turn based on sensor readings'''
    global OBSTACLE_AVOIDING
    OBSTACLE_AVOIDING = True  # Set flag to indicate obstacle avoidance is active
    #print("Starting obstacle avoidance...")

    left_sensor_reading = readings.get('u1', float('inf'))  # Left sensor
    right_sensor_reading = readings.get('u2', float('inf'))  # Right sensor
    front_sensor_reading = readings.get('u0', float('inf'))  # Front sensor
    back_sensor_reading = readings.get('u3', float('inf'))  # Back sensor

    # Step 1: Reverse if front sensor detects an obstacle
    if front_sensor_reading < ULTRASONIC_THRESHOLD:
        #print(f"Obstacle detected ahead. Reversing by {REVERSE_DISTANCE} inches.")
        reverse_packet = packetize(f"w0:-{REVERSE_DISTANCE}")
        if reverse_packet:
            transmit(reverse_packet)
            result = receive()
            if result is not None:
                [responses, _] = result
                #print(f"Reverse command response: {response_string(reverse_packet, responses)}")
    if back_sensor_reading < ULTRASONIC_THRESHOLD:
        #print(f"Obstacle detected behide. forward by {REVERSE_DISTANCE} inches.")
        reverse_packet = packetize(f"w0:{REVERSE_DISTANCE}")
        if reverse_packet:
            transmit(reverse_packet)
            result = receive()
            if result is not None:
                [responses, _] = result
                #print(f"forward command response: {response_string(reverse_packet, responses)}")

    # Step 2: Lateral adjustment for left or right sensor
    if left_sensor_reading < ULTRASONIC_THRESHOLD:
        #print("Obstacle detected on the left. Turning right slightly and moving back.")
        # Turn right slightly
        turn_right_packet = packetize('d0:3')  # Adjust the angle as needed
        if turn_right_packet:
            transmit(turn_right_packet)
            result = receive()
            if result is not None:
                [responses, _] = result
                #print(f"Turn right response: {response_string(turn_right_packet, responses)}")

    # If the right sensor detects an obstacle, turn slightly left and reverse to simulate a move away
    elif right_sensor_reading < ULTRASONIC_THRESHOLD:
        #print("Obstacle detected on the right. Turning left slightly and moving back.")
        # Turn left slightly
        turn_left_packet = packetize('d0:-3')  # Adjust the angle as needed
        if turn_left_packet:
            transmit(turn_left_packet)
            result = receive()
            if result is not None:
                [responses, _] = result
                #print(f"Turn left response: {response_string(turn_left_packet, responses)}")

    OBSTACLE_AVOIDING = False  # Reset flag after obstacle avoidance


def sensor_checking_loop():
    '''Continuously check the ultrasonic sensors and invoke obstacle avoidance when needed.'''
    global OBSTACLE_AVOIDING
    while RUN_COMMUNICATION_CLIENT:
        readings = check_ultrasonic_sensors()
        if readings and any(value < ULTRASONIC_THRESHOLD for value in readings.values()):
            if not OBSTACLE_AVOIDING:
                avoid_obstacle(readings)
        time.sleep(0.1)  # Check sensors every 0.1 seconds


############## Main Execution Starts Here ##############
if __name__ == "__main__":
    # Start the command execution loop and sensor checking in parallel using threads
    sensor_thread = threading.Thread(target=sensor_checking_loop, daemon=True)
    sensor_thread.start()

    # Run the main command execution loop
    command_execution_loop()









############## Main section for the open loop control algorithm ##############
# The sequence of commands to run
CMD_SEQUENCE = ['w0:36', 'r0:90', 'w0:36', 'r0:90', 'w0:12', 'r0:-90', 'w0:24', 'r0:-90', 'w0:6', 'r0:720']  # Move forward 360 inches
LOOP_PAUSE_TIME = 0.25  # seconds

RUN_DEAD_RECKONING = False  # If true, run this. If false, skip it
ct = 0

############## Main Loop ##############
while RUN_DEAD_RECKONING:
    # Check ultrasonic sensors for obstacles
    readings = check_ultrasonic_sensors()
    if readings and any(value < ULTRASONIC_THRESHOLD for value in readings.values()):
        avoid_obstacle(readings)  # Avoid the obstacle based on sensor readings
        continue  # After avoidance, recheck and continue

    # Execute the next command in the sequence
    if ct < len(CMD_SEQUENCE):
        packet_tx = packetize(CMD_SEQUENCE[ct])
        if packet_tx:
            transmit(packet_tx)
            result = receive()
            if result is not None:
                [responses, time_rx] = result
                print(f"Drive command response: {response_string(CMD_SEQUENCE[ct], responses)}")

            if responses[0] and responses[0][1] == 'True':
                ct += 1
    else:
        RUN_DEAD_RECKONING = False
        print("Sequence complete!")

    time.sleep(LOOP_PAUSE_TIME)



