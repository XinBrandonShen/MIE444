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

import socket
import time
from datetime import datetime
import serial
import threading

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




############## Global Flags ##############
OBSTACLE_AVOIDING = False  # Flag to control when obstacle avoidance is running
RUN_DEAD_RECKONING = True  # Main loop running flag

############## Main section for the communication client ##############
RUN_COMMUNICATION_CLIENT = False  # If true, run this. If false, skip it
while RUN_COMMUNICATION_CLIENT:
    # Input a command
    cmd = input('Type in a string to send: ')

    # Send the command
    packet_tx = packetize(cmd)
    if packet_tx:
        transmit(packet_tx)

    # Receive the response
    result = receive()
    if result is not None:
        [responses, time_rx] = result
        # Check if responses is a list and not a float
        if isinstance(responses, list) and responses[0]:
            print(f"At time '{time_rx}' received from {SOURCE}:\n{response_string(cmd, responses)}")
        else:
            print(f"At time '{time_rx}' received from {SOURCE}:\nMalformed Packet")
    else:
        print("No response received or connection issue occurred.")


ULTRASONIC_THRESHOLD = 2  # Threshold for obstacle detection (in inches)
LATERAL_ADJUST_DISTANCE = 1  # Distance to move laterally when avoiding obstacles

############## Sensor Checking Function ##############
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
                print(f"Ultrasonic sensor {sensor_id} reading: {sensor_reading} inches")
            else:
                print(f"Failed to receive a response from sensor {sensor_id}.")
    return readings

############## Lateral Adjustment Function ##############
def lateral_avoidance():
    '''Continuously checks sensors and moves the rover laterally to avoid obstacles.'''
    global OBSTACLE_AVOIDING
    while True:
        readings = check_ultrasonic_sensors()  # Continuously read sensors
        left_sensor_reading = readings.get('u1', float('inf'))  # Left sensor
        right_sensor_reading = readings.get('u2', float('inf'))  # Right sensor
        front_sensor_reading = readings.get('u0', float('inf'))  # Front sensor

        # Move right if there's an obstacle on the left
        if left_sensor_reading < ULTRASONIC_THRESHOLD:
            print("Obstacle detected on the left. Moving right.")
            OBSTACLE_AVOIDING = True
            move_right_packet = packetize(f"m0:{LATERAL_ADJUST_DISTANCE}")  # Move right
            if move_right_packet:
                transmit(move_right_packet)
                result = receive()
                if result is not None:
                    [responses, _] = result
                    print(f"Lateral move right response: {response_string(move_right_packet, responses)}")
                print("Rover moved right to avoid obstacle on the left.")

            OBSTACLE_AVOIDING = False

        # Move left if there's an obstacle on the right
        elif right_sensor_reading < ULTRASONIC_THRESHOLD:
            print("Obstacle detected on the right. Moving left.")
            OBSTACLE_AVOIDING = True
            move_left_packet = packetize(f"m0:-{LATERAL_ADJUST_DISTANCE}")  # Move left
            if move_left_packet:
                transmit(move_left_packet)
                result = receive()
                if result is not None:
                    [responses, _] = result
                    print(f"Lateral move left response: {response_string(move_left_packet, responses)}")
                print("Rover moved left to avoid obstacle on the right.")

            OBSTACLE_AVOIDING = False

        # Check if there's an obstacle in the front
        elif front_sensor_reading < ULTRASONIC_THRESHOLD:
            print("Obstacle detected in the front. Reversing.")
            OBSTACLE_AVOIDING = True
            reverse_packet = packetize(f"w0:-{LATERAL_ADJUST_DISTANCE}")  # Reverse slightly
            if reverse_packet:
                transmit(reverse_packet)
                result = receive()
                if result is not None:
                    [responses, _] = result
                    print(f"Reverse command response: {response_string(reverse_packet, responses)}")
                print("Rover reversed to avoid obstacle in the front.")
            OBSTACLE_AVOIDING = False

        else:
            print("No obstacles detected. Moving forward.")
            forward_packet = packetize(f"w0:6")  # Move forward if no obstacles
            if forward_packet:
                transmit(forward_packet)
                result = receive()
                if result is not None:
                    [responses, _] = result
                    print(f"Forward command response: {response_string(forward_packet, responses)}")
            print("Rover moving forward as no obstacles detected.")
        
        time.sleep(0.5)  # Check sensors every 0.5 seconds

############## Main Loop for Command Execution ##############
CMD_SEQUENCE = ['w0:36', 'r0:90', 'w0:36', 'r0:90', 'w0:12', 'r0:-90', 'w0:24', 'r0:-90', 'w0:6', 'r0:720']
LOOP_PAUSE_TIME = 0.25  # seconds

############## Main Loop for Command Execution ##############
def main_movement():
    '''Executes the main movement commands in the CMD_SEQUENCE.'''
    global OBSTACLE_AVOIDING
    ct = 0
    while ct < len(CMD_SEQUENCE):
        if not OBSTACLE_AVOIDING:  # Execute main command only if not avoiding obstacles
            packet_tx = packetize(CMD_SEQUENCE[ct])
            if packet_tx:
                transmit(packet_tx)
                result = receive()

                # Ensure the result is valid and not a float or malformed data
                if result is not None:
                    [responses, time_rx] = result
                    if isinstance(responses, list) and responses[0]:
                        print(f"Drive command response: {response_string(CMD_SEQUENCE[ct], responses)}")

                        if responses[0][1] == 'True':
                            ct += 1
                    else:
                        print("Malformed response or empty packet received.")
                else:
                    print("No response received or connection issue occurred.")
            
            time.sleep(LOOP_PAUSE_TIME)  # Pause between commands
        else:
            time.sleep(0.1)  # If obstacle avoidance is happening, wait before checking the next command



############## Main Execution Starts Here ##############
if __name__ == "__main__":
    # Define the CMD_SEQUENCE or load it from external sources
    CMD_SEQUENCE = ['w0:36', 'r0:90', 'w0:36', 'r0:90', 'w0:12', 'r0:-90', 'w0:24', 'r0:-90', 'w0:6', 'r0:720']

    # Start two threads: one for obstacle avoidance and one for the main movement
    obstacle_thread = threading.Thread(target=lateral_avoidance, daemon=True)
    movement_thread = threading.Thread(target=main_movement, daemon=True)

    # Start both threads
    obstacle_thread.start()
    movement_thread.start()

    # Join threads (wait for them to finish)
    movement_thread.join()  # The program will keep running until the main movement completes
