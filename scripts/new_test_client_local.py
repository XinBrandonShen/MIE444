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
import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import convolve
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
# stop the rover
def stop_rover():
    packet_tx = packetize('xx')  # Send the stop command
    transmit(packet_tx)
    print("Rover stopped.")
    
# Check Ultrasonic Sensor
def check_ultrasonic(sensor_id):
    packet_tx = packetize(sensor_id)
    if packet_tx:
        transmit(packet_tx)
        [responses, time_rx] = receive()
        print(f"Ultrasonic reading: {response_string(sensor_id, responses)}")

        # Check if the response is numeric
        try:
            # Attempt to evaluate if it's a number
            distance = eval(responses[0][1])
            return distance  # Return the numeric distance
        except (SyntaxError, NameError):
            # If eval fails due to a non-numeric response, handle gracefully
            print(f"Warning: Non-numeric response '{responses[0][1]}' received for sensor {sensor_id}")
            return float('0')  # Return a large value if sensor fails or non-numeric response

    return float('0')  # Return a large value if packetization fails


def execute_command(command):
    packet_tx = packetize(command)
    if packet_tx:
        transmit(packet_tx)
        [responses, time_rx] = receive()
        print(f"Executed command: {command}, Drive command response: {response_string(command, responses)}")
    
    
# Execute turning functions using the execute_command logic
def turn_180():
    global last_turn
    command = 'e0:180'  # Command to turn 180 degrees
    execute_command(command)
    last_turn = "180"
    time.sleep(3)
    CMD = command
    return CMD

def turn_left():
    global last_turn
    command = 'e0:-90'  # Command to turn left (90 degrees)
    execute_command(command)
    last_turn = "left"
    time.sleep(1)
    CMD = command
    return CMD

def turn_right():
    global last_turn
    command = 'e0:90'  # Command to turn right (90 degrees)
    execute_command(command)
    last_turn = "right"
    time.sleep(1)
    CMD = command
    return CMD
    
def align_to_nearest_90():
    current_angle = check_ultrasonic("c0")  # Get the current angle from the compass sensor
    
    # Find the nearest 90-degree based angle (0, 90, 180, or 270)
    target_angle = round(current_angle / 90) * 90
    
    # Calculate the minimal rotation needed to align with the target angle
    rotation_amount = target_angle - current_angle
    
    # Adjust rotation direction if the angle difference is more than 180 degrees
    if rotation_amount > 180:
        rotation_amount -= 360
    elif rotation_amount < -180:
        rotation_amount += 360
    
    # Rotate the rover by the calculated amount
    if rotation_amount > 0:
        print(f"Rotating clockwise by {rotation_amount} degrees to align with {target_angle}°.")
        CMD = 'e0:' + str((rotation_amount))
        execute_command(CMD)
    elif rotation_amount < 0:
        print(f"Rotating counterclockwise by {-rotation_amount} degrees to align with {target_angle}°.")
        CMD = 'e0:' + str((-rotation_amount))
        execute_command(CMD)
    else:
        print("Rover is already aligned with the nearest 90° angle.")
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


### Pause time after sending messages
if SIMULATE:
    TRANSMIT_PAUSE = 0.1
else:
    TRANSMIT_PAUSE = 0



#### Auto Obstacle Avoidance

#### Localization #####


dim1=32
dim2=16

    # Create a black and white random map (0 = black, 1 = white)
n = dim1 * dim2
np.random.seed(5489)
bw = np.random.randint(0, 2, (dim2, dim1))

    # Create the blocks based on predefined positions
M = np.zeros_like(bw)
blocks = np.array([
        [2, 3], [3, 2], [4, 3], [5, 1],
        [5, 3], [7, 1], [7, 3], [7, 4]
])
for x, y in blocks:
    M[(y - 1) * 4:(y - 1) * 4 + 4, (x - 1) * 4:(x - 1) * 4 + 4] = 1

    # Add walls around the map
M = np.pad(M, pad_width=1, mode='constant', constant_values=1)

    # Generate ultrasonic world
ultra = np.zeros_like(bw)
for sec_row in range(0, dim2, 4):
    for sec_col in range(0, dim1, 4):
        seg_row = M[sec_row + 2, sec_col:sec_col + 6]
        seg_col = M[sec_row:sec_row + 6, sec_col + 2]
        val = np.sum(seg_row) + np.sum(seg_col)
        if val == 2 and np.sum(seg_row) != 1:
            val = 5
        ultra[sec_row:sec_row + 4, sec_col:sec_col + 4] = val

    # Create mask for blocks
M = 1 - M[1:-1, 1:-1]

p = np.ones((dim2, dim1)) / n

def detect_and_update(u_digital, ultra, mask, p, heading, command):

    # Sum the digital sensor readings
    m_u = sum(u_digital)

    # Detect patterns for adjacent and opposite blocking
    if m_u == 2:
        # Check for opposite blocking (front-back or left-right)
        if (u_digital[0] and u_digital[3]) or (u_digital[1] and u_digital[2]):
            m_u = 5  # Opposite blocking
        else:
            m_u = 2  # Adjacent blocking

    print('m_u is:', m_u)

    # Sensor update using the detected value
    p = sense_u(ultra, mask, p, m_u)

    # Execute the movement update based on the command
    p, heading = move(p, mask, heading, command)

    return p, heading


def sense_u(world, mask, p, SenVal):
    """
    Sensor update function equivalent to sense_u in MATLAB.
    - world: Ultrasonic world matrix
    - mask: Obstacle mask matrix
    - p: Current probability distribution matrix
    - SenVal: Sensor reading value
    """
    pHit = 0.9
    pMiss = 0.1

    # Create a multiplier matrix based on sensor readings
    mult = np.full(world.shape, pMiss)
    mult[world == SenVal] = pHit

    # Element-wise multiplication of probability matrix with sensor model
    pnew = p * mult

    # Apply the obstacle mask
    pnew = pnew * mask

    # Normalize the probabilities
    pnew /= np.sum(pnew)
    return pnew


def move(p, mask, heading, Command):
    """
    Movement update function equivalent to move.m in MATLAB.
    - p: Current probability distribution matrix
    - mask: Obstacle mask matrix
    - heading: Current heading of the rover (in degrees)
    - Move: Movement command ('w', 'a', 's', 'd')
    """
    # Movement error model using a 3x3 convolution kernel
    K = np.array([[0.05, 0.1, 0.05],
                  [0., 0.7, 0.1],
                  [0.05, 0.1, 0.05]])
    
    # Convolution to model uncertainty
    pnew = np.convolve(p.flatten(), K.flatten(), mode='same').reshape(p.shape)
    
    command_type, amount = Command.split(':')
    amount = int(amount)

    col_move = 0
    row_move = 0
    # Adjust heading based on command type
    if command_type == 'e0':  # Rotate
        heading = (heading - amount) % 360
        lateral_heading = heading

    elif command_type == 'w0':  # Forward/backward
        lateral_heading = (heading) if amount > 0 else (heading + 180)
        col_move = int(np.cos(np.deg2rad(lateral_heading)))
        row_move = int(np.sin(np.deg2rad(lateral_heading)))    
    elif command_type == 'd0':  # Left/right relative to heading
        lateral_heading = (heading + 90) if amount > 0 else (heading - 90)
        col_move = int(np.cos(np.deg2rad(lateral_heading)))
        row_move = int(np.sin(np.deg2rad(lateral_heading)))


    print('heading: ', heading)
    print('lateral heading: ', lateral_heading)
    print('col:', col_move)
    print('row:', row_move)
    # Shift the probability distribution based on movement direction
    if col_move > 0:
        pnew = np.hstack([np.zeros((pnew.shape[0], 1)), pnew[:, :-1]])
    elif col_move < 0:
        pnew = np.hstack([pnew[:, 1:], np.zeros((pnew.shape[0], 1))])

    if row_move > 0:
        pnew = np.vstack([pnew[1:, :], np.zeros((1, pnew.shape[1]))])
    elif row_move < 0:
        pnew = np.vstack([np.zeros((1, pnew.shape[1])), pnew[:-1, :]])

    # Apply the obstacle mask
    pnew = pnew * mask

    # Normalize the probabilities
    if np.sum(pnew) > 0:
        pnew /= np.sum(pnew)

    return pnew, heading




######## Navigation ########

############## Main section for the communication client ##############
RUN_COMMUNICATION_CLIENT = False # If true, run this. If false, skip it
heading = 90
n = dim1 * dim2
p = np.ones((dim2, dim1)) / n
target_position = (2, 2)  # Target location to navigate to (top-left square)
confidence_threshold = 0.25  # Confidence threshold to stop localization
localized = False
current_pos = None


while RUN_COMMUNICATION_CLIENT:
    # Input a command manually
    cmd = input('Type in a string to send: ')
    
    # Retrieve sensor values
    front_dist = check_ultrasonic('u0')
    right_dist = check_ultrasonic('u1')
    left_dist = check_ultrasonic('u2')
    back_dist = check_ultrasonic('u3')
    angle = check_ultrasonic("c0")

    # Process sensor readings to detect patterns
    u_digital = [front_dist <= 8, right_dist <= 8, left_dist <= 8, back_dist <= 8]
    m_u = sum(u_digital)
    if m_u == 2:
        if (u_digital[0] and u_digital[3]) or (u_digital[1] and u_digital[2]):
            m_u = 5  # Opposite blocking
        else:
            m_u = 2  # Adjacent blocking

    print('m_u is:', m_u)

    # Update the probability map with the sensor value
    if not localized:
        p = sense_u(ultra, M, p, m_u)
        
        # Check if localization is confident enough
        if is_localized(p, confidence_threshold):
            localized = True
            current_pos = np.unravel_index(np.argmax(p), p.shape)
            print(f"Localization successful! Current Position: {current_pos}")
            
            # Hardcode the navigation route to the target position
            navigation_route = navigate_to_target(p, heading, current_pos, target_position)
            print("Generated navigation route:", navigation_route)
            
            # Start executing the navigation route
            for nav_cmd in navigation_route:
                p, heading = move(p, M, heading, nav_cmd)
                print(f"Executing navigation command: {nav_cmd}")
                
                # Plot the updated probability map after each move
                plt.clf()
                plt.imshow(p, cmap='hot', interpolation='nearest')
                plt.colorbar(label="Probability")
                plt.title(f"Navigating to Target (Heading: {heading}°, Command: {nav_cmd})")
                plt.pause(0.1)
            print("Navigation complete!")
            break  # Exit the loop after reaching the target position

    # Continue manual movement updates if not localized
    if not localized and cmd != 'xx':
        p, heading = move(p, M, heading, cmd)

    # Send the command via communication channel
    packet_tx = packetize(cmd)
    if packet_tx:
        transmit(packet_tx)

    # Receive and display the response
    responses, time_rx = receive()
    if responses[0]:
        print(f"At time '{time_rx}' received from {SOURCE}:\n{response_string(cmd, responses)}")
    else:
        print(f"At time '{time_rx}' received from {SOURCE}:\nMalformed Packet")

    # Plot the probability map during localization
    plt.clf()
    plt.imshow(p, cmap='hot', interpolation='nearest')
    plt.colorbar(label="Probability")
    plt.title(f"Localization in Progress (Heading: {heading}°, Command: {cmd})")
    plt.pause(0.1)

















############## Main section for the open loop control algorithm ##############


############## Main section for the open loop control algorithm ##############
heading = 90
ct = 0
dis_to_wall = 0
dis_2_stop = 5
d_safe_L_or_R = 1
Tolerance_front = 0
Maze_Block_Size = 12
Wall_Align_Correction = True

n = dim1 * dim2
p = np.ones((dim2, dim1)) / n
target_position = (2, 2)  # Target location to navigate to (top-left square)
confidence_threshold = 0.25  # Confidence threshold to stop localization
localized = False
final_destination = False
current_pos = None
# Main loop
RUN_DEAD_RECKONING = True # If true, run this. If false, skip it
# packet_tx1_bull = False

# Auto Obstacle Avoidance Loop
Moving = False # If the rover is moving set to be true, otherwise set to be false

while RUN_DEAD_RECKONING:

    # Retrieve sensor readings
    front_dist = check_ultrasonic('u0')
    right_dist = check_ultrasonic('u1')
    left_dist = check_ultrasonic('u2')
    back_dist = check_ultrasonic('u3')
    angle = check_ultrasonic("c0")


    if localized == False:

        if front_dist <= dis_2_stop:
            CMD = 'xx'
            execute_command(CMD)
            print('front obstacle, stop rover')
            print("Front Obstacle detected! Stopping the rover.")
            if left_dist < Maze_Block_Size and right_dist < Maze_Block_Size:
                print("Both left and right are blocked. Turning 180 degrees.")
                CMD = turn_180()

            elif left_dist < Maze_Block_Size:
                print("Left is blocked, turning right.")
                CMD = turn_right()
  
            elif right_dist < Maze_Block_Size:
                print("Right is blocked, turning left.")
                CMD = turn_left()

            else:
                if left_dist > right_dist:
                    print("Neither side blocked, left path is longer. Turning left.")
                    CMD = turn_left()

                else:
                    print("Neither side blocked, right path is longer. Turning right.")
                    CMD = turn_right()
            u_digital = [front_dist <= 8, right_dist <= 8, left_dist <= 8, back_dist <= 8]
            p, heading = detect_and_update(u_digital, ultra, M, p, heading, CMD)

        # Side obstacles
        elif right_dist <= d_safe_L_or_R:
            stop_rover()
            CMD = 'd0:3'
            execute_command(CMD)
            print("Slightly moving left to avoid right wall.")
        elif left_dist <= d_safe_L_or_R:
            stop_rover()
            CMD = 'd0:-3'
            execute_command(CMD)
            print("Slightly moving right to avoid left wall.")

        # Back obstacle
        elif back_dist <= dis_2_stop:
            stop_rover()
            CMD = 'w0:3'
            execute_command(CMD)
            print("Back obstacle detected. Moving forward.")

        # No obstacles detected, move forward
        if not Moving:
            CMD = 'w0:3'
            execute_command(CMD)
            print(f"Moving forward by {CMD.split(':')[1]} units.")
        u_digital = [front_dist <= 8, right_dist <= 8, left_dist <= 8, back_dist <= 8]
        p, heading = detect_and_update(u_digital, ultra, M, p, heading, CMD)
        max_pos = np.unravel_index(np.argmax(p), p.shape)
        row, col = max_pos
        max_value = p[row, col]
        print('max probability is: ', max_value)

        time.sleep(0.2)

        if row < 6 and col < 6 and max_value > 0.1:
            print(f"Current position is at (row: {row},coloumn: {col}), Loading Zone Reached!")
            stop_rover()
            localized = True   
            fig, ax = plt.subplots()
            # Remove the axis for a cleaner look
            ax.axis('off')
            # Define the text content
            text_content = """
            Loading Zone reached!
            """
            ax.text(
                0.5, 0.5,               # Position (centered)
                text_content,           # Text content
                fontsize=12,            # Font size
                ha='center',            # Horizontal alignment
                va='center',            # Vertical alignment
                bbox=dict(facecolor='lightblue', alpha=0.5, boxstyle='round,pad=1')  # Box style
            )
            plt.pause(2)
            plt.show(block = False)
            plt.close()
        else:
            print(f"Max probability is {max_value}, Current position is at ('row: ',{row},'coloumn: ', {col}), not yet at destination.")
    



    if localized == True:
        print('location: ', row, col)
        

        if heading!=90:
        # first turn the heading toward north
            desired_heading = 90
                # Calculate the shortest rotation to face up
            rotation_amount = (desired_heading - heading) % 360
            # Determine if clockwise or counter-clockwise rotation is shorter
            if rotation_amount <= 180:
                # Clockwise rotation
                command = f"e0:{rotation_amount}"
            else:
                # Counter-clockwise rotation
                rotation_amount = 360 - rotation_amount
                command = f"e0:-{rotation_amount}"
            execute_command(CMD)
            # Print the command to face up
            print(f"Command to face up: {command}")
        
        print(f"Current position is at (row: {row},coloumn: {col})")
        if row <= 2 and col <= 4:  
            CMD = 'd0:-3'
            print('rover stays within the block 1,1, move rover RIGHT a block distance')
            execute_command(CMD)
            col_change = 1
        if row <= 2 and 4 < col <= 8:
            CMD = 'd0:-3'
            print('rover stays within the block 1,2, move rover RIGHT a block distance')
            execute_command(CMD)
            col_change = 1
        if row <= 4 and 8 < col <= 12:
            CMD = 'd0:-4'
            print('rover stays within the block 1,3, move rover RIGHT a block distance')
            execute_command(CMD)
            col_change = 1
        if row <= 4 and 12 < col <= 16:
            CMD = 'w0:-3'
            print('rover stays within the block 1,4, move rover DOWN a block distance')
            execute_command(CMD)
            row_change = 1
        if 4 < row <= 8 and 12 < col <= 16:
            CMD = 'd0:-3'
            print('rover stays within the block 2,4, move rover RIGHT a block distance')
            execute_command(CMD)
            col_change = 1
        if 4 < row <= 8 and 17 < col <= 20:
            CMD = 'd0:-4'
            print('rover stays within the block 2,5, move rover RIGHT a block distance')
            execute_command(CMD)
            col_change = 1
        if 4 < row <= 8 and 21 < col <= 24:
            CMD = 'w0:3'
            print('rover stays within the block 2,6, move rover UP a block distance')
            execute_command(CMD)
            row_change = -1
        if 2 < row <= 7 and  col <= 4:
            CMD = 'w0:3'
            print('rover stays within the block 2,1, move rover UP a block distance')
            execute_command(CMD)
            row_change = -1
        if 2 < row <= 7 and 4 < col <= 8:
            CMD = 'w0:3'
            print('rover stays within the block 2,2, move rover UP a block distance')
            execute_command(CMD)
            row_change = -1

        front_dist = check_ultrasonic('u0')
        right_dist = check_ultrasonic('u1')
        left_dist = check_ultrasonic('u2')
        back_dist = check_ultrasonic('u3')
        angle = check_ultrasonic("c0")

        u_digital = [front_dist <= 8, right_dist <= 8, left_dist <= 8, back_dist <= 8]
        p, heading = detect_and_update(u_digital, ultra, M, p, heading, CMD)
        max_pos = np.unravel_index(np.argmax(p), p.shape)
        max_value = p[row, col]
        print('max probability is: ', max_value)
        row, col = max_pos

        if  row <= 2 and 20 < col <= 30:
            print('Final Destination Reached!!!!')
            stop_rover()
            fig, ax = plt.subplots()

            # Remove the axis for a cleaner look
            ax.axis('off')

            # Define the text content
            text_content = """
            Final Destination Zone reached!
            """
            # Add the text box
            ax.text(
                0.5, 0.5,               # Position (centered)
                text_content,           # Text content
                fontsize=12,            # Font size
                ha='center',            # Horizontal alignment
                va='center',            # Vertical alignment
                bbox=dict(facecolor='lightblue', alpha=0.5, boxstyle='round,pad=1')  # Box style
            )
            plt.pause(2)
            plt.show(block = False)
            break
            
     
    # Packet transmission for other sensor readings
    packet_tx = packetize('g0,c0,i0')
    if packet_tx:
        transmit(packet_tx)
        [responses, time_rx] = receive()
        print(f"Other sensor readings:\n{response_string('g0,c0,i0', responses)}")

    # Display the updated probability map
    plt.figure(1)
    plt.clf()
    plt.imshow(p, cmap='hot', interpolation='nearest')
    plt.colorbar(label="Probability")
    plt.title(f"Updated Localization Probability Map (Heading: {heading}°, Command: {CMD})")
    plt.pause(0.1)
    plt.show(block=False)

        
    
    
