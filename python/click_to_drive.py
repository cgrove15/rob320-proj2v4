import socket
import numpy as np
import threading
import signal
import pygame
from argparse import ArgumentParser
import logging
import errno

import sys
sys.path.append('./')

from rixmsg.standard.UInt32 import UInt32
from rixmsg.sensor.LaserScan import LaserScan
from rixmsg.geometry.Pose2DStamped import Pose2DStamped
from rixmsg.geometry.Twist2DStamped import Twist2DStamped

mbot_mutex = threading.Lock()
current_pose = None
current_pose_mutex = threading.Lock()
current_scan = None
current_scan_mutex = threading.Lock()

mbot_img = pygame.transform.scale(pygame.image.load("mbot-omni.png"), (50, 50))
pgon = None
pose_trail = []

logging.basicConfig(level=logging.INFO, format='[%(asctime)s] [%(levelname)s] [%(message)s]')

def update_plot(screen):
    global current_scan, current_scan_mutex, current_pose, current_pose_mutex, mbot_img, pgon

    if not current_scan or not current_pose:
        return

    screen.fill((255, 255, 255))  # Clear screen with white

    # Filter ranges > 0
    with current_scan_mutex:
        valid_indices = np.intersect1d(np.where(np.array(current_scan.ranges) > 0), np.where(np.array(current_scan.intensities) > 0))
        valid_ranges = np.array(current_scan.ranges)[valid_indices]
        valid_thetas = np.arange(current_scan.angle_min, current_scan.angle_max, current_scan.angle_increment)[valid_indices]

    center_x, center_y, robot_theta = 512, 512, 0
    with current_pose_mutex:
        if current_pose is not None:
            center_x = 512 + int(current_pose.pose.x * 100)
            center_y = 512 - int(current_pose.pose.y * 100)
            robot_theta = current_pose.pose.theta

    edge_points = []
    for theta, r in zip(valid_thetas, valid_ranges):
        end_x = int(center_x + r * 100 * np.cos(theta))
        end_y = int(center_y - r * 100 * np.sin(theta))
        edge_points.append((end_x, end_y))

    # Draw the outline connecting the outer edge of the rays
    if edge_points:
        pgon = pygame.draw.polygon(screen, (255, 201, 11), edge_points)
        pygame.draw.polygon(screen, (0, 35, 76), edge_points, 3)

    # Add current pose to the plot as label
    with current_pose_mutex:
        if current_pose is not None:
            monospace_font_name = pygame.font.match_font('couriernew', 'consolas', 'dejavusansmono')
            font = pygame.font.Font(monospace_font_name, 18)
            font.bold = True
            pose_text = f"x: {current_pose.pose.x}\ny: {current_pose.pose.y}\nθ: {current_pose.pose.theta}"
            lines = pose_text.split('\n')
            y_offset = 25  # Place text below the bounding box
            pygame.draw.rect(screen, (255, 255, 255), (10, 10, 300, 100))
            pygame.draw.rect(screen, (0, 0, 0), (10, 10, 300, 100), 2)
            for line in lines:
                text_surface = font.render(line, True, (0, 0, 0))
                screen.blit(text_surface, (25, y_offset))
                y_offset += text_surface.get_height()

            pose_trail.append((center_x, center_y))
            if len(pose_trail) > 1000:
                pose_trail.pop(0)
            
            if len(pose_trail) > 1:
                pygame.draw.lines(screen, (154, 51, 36), False, pose_trail, 4)

    # Plot the robot (mbot-omni.png)
    mbot_img_transformed = pygame.transform.rotate(mbot_img, np.degrees(robot_theta))
    robot_rect = mbot_img_transformed.get_rect(center=(center_x, center_y))
    screen.blit(mbot_img_transformed, robot_rect)

    pygame.display.flip()

def mbot_thread(mbot_client):
    global ctrl_c_pressed, current_pose
    while True:
        bytes_received = 0
        try:
            msgLen = UInt32()
            recvSize = msgLen.size()
            msgLenBuffer = bytearray(recvSize)
            mbot_client.recv_into(msgLenBuffer, recvSize)
            msgLen.deserialize(msgLenBuffer, {"offset": 0})
            msgBuffer = bytearray(msgLen.data)
            # print("Reading " + str(msgLen.data) + " bytes")
            bytesRecv = 0
            while bytesRecv < msgLen.data:
                bytesRecv += mbot_client.recv_into(
                    memoryview(msgBuffer)[bytesRecv:], msgLen.data - bytesRecv
                )

            new_pose = Pose2DStamped()
            new_pose.deserialize(msgBuffer, {"offset": 0})

            with current_pose_mutex:
                current_pose = new_pose
                # logging.info(f"Received pose.")

        except Exception as e:
            # if e.errno == errno.EWOULDBLOCK:
            #     continue
            logging.error(f"Error receiving data from mbot_client: {e}")
            break

    logging.info("MBot thread exiting")

def lidar_thread(lidar_client):
    global ctrl_c_pressed, current_scan, current_scan_mutex
    while True:
        bytes_received = 0
        try:
            msgLen = UInt32()
            recvSize = msgLen.size()
            msgLenBuffer = bytearray(recvSize)
            lidar_client.recv_into(msgLenBuffer, recvSize)
            msgLen.deserialize(msgLenBuffer, {"offset": 0})
            msgBuffer = bytearray(msgLen.data)
            print("Reading " + str(msgLen.data) + " bytes")
            bytesRecv = 0
            while bytesRecv < msgLen.data:
                bytesRecv += lidar_client.recv_into(
                    memoryview(msgBuffer)[bytesRecv:], msgLen.data - bytesRecv
                )

            new_scan = LaserScan()
            new_scan.deserialize(msgBuffer, {"offset": 0})

            with current_scan_mutex:
                current_scan = new_scan
                logging.info(f"Received scan.")
        
        except Exception as e:
            # if e.errno == errno.EWOULDBLOCK:
            #     continue
            logging.error(f"Error receiving data from lidar_client: {e}")
            break

    logging.info("Lidar thread exiting")

def main():
    global ctrl_c_pressed, pgon, current_pose, current_pose_mutex, current_scan, current_scan_mutex

    parser = ArgumentParser()
    parser.add_argument("address", type=str)
    args = parser.parse_args()

    # Create a socket object
    lidar_client = socket.socket()
    mbot_client = socket.socket()
    pose_command_server = socket.socket()

    # connect to the server on local computer
    address = args.address
    lidar_port = 8100
    mbot_port = 8200
    pose_command_port = 8300

    pose_command_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    pose_command_server.bind(("0.0.0.0", pose_command_port))
    pose_command_server.listen()

    logging.info(f"Connecting to Lidar server at {address}:{lidar_port}")
    try:
        lidar_client.connect((address, lidar_port))
    except socket.timeout:
        logging.error("Timeout occurred while connecting to Lidar server")
        return
    except Exception as e:
        logging.error(f"Error connecting to Lidar server: {e}")
        return
    logging.info("Connected to Lidar server")

    logging.info(f"Connecting to MBot server at {address}:{mbot_port}")
    try:
        mbot_client.connect((address, mbot_port))
    except socket.timeout:
        logging.error("Timeout occurred while connecting to MBot server")
        return
    except Exception as e:
        logging.error(f"Error connecting to MBot server: {e}")
        return
    logging.info("Connected to MBot server")
    
    pose_command_connection = None
    logging.info(f"Accepting connection on Pose Command Server")
    try:
        pose_command_connection = pose_command_server.accept()[0]
    except socket.timeout:
        logging.error("Timeout occurred while connecting to MBot server")
        return
    except Exception as e:
        logging.error(f"Error connecting to MBot server: {e}")
        return
    logging.info("Accepted connection.")

    # Initialize pygame
    pygame.init()
    screen = pygame.display.set_mode((1024, 1024))
    pygame.display.set_caption('Click to Drive')

    # Create the MBot thread
    mbot_thread_handle = threading.Thread(target=mbot_thread, args=(mbot_client,), daemon=True)
    mbot_thread_handle.start()

    # Create the Lidar thread
    lidar_thread_handle = threading.Thread(target=lidar_thread, args=(lidar_client,), daemon=True)
    lidar_thread_handle.start()

    # Plot lidar scan in main process (pygame is not thread safe)
    while True:
        try:
            for event in pygame.event.get():
                # Handle quit event
                if event.type == pygame.QUIT:
                    ctrl_c_pressed = True
                # Handle mouse click event
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    mouse_x, mouse_y = event.pos

                    # Check if the click is inside the polygon (inside the lidar scan)
                    if pgon:
                        r,g,b,_ = tuple(screen.get_at((mouse_x, mouse_y)))
                        if r != 255 or g != 201 or b != 11:
                            continue

                    # Convert to world coordinates in meters
                    pose = Pose2DStamped()
                    with current_pose_mutex:
                        pose.pose.x =  (mouse_x - 512) / 100
                        pose.pose.y = -(mouse_y - 512) / 100

                    # Send the drive command
                    pose.pose.theta = 0
                    
                    # Serialize the command
                    pose_size = UInt32()
                    pose_size.data = pose.size()
                
                    pose_serialized = bytearray()
                    pose_size.serialize(pose_serialized)
                    pose.serialize(pose_serialized)

                    with mbot_mutex:
                        logging.info(f"Sending pose command: {pose}")
                        pose_command_connection.send(pose_serialized)

            update_plot(screen)
        except Exception as e:
            break

    if mbot_thread_handle.is_alive():
        logging.info("Waiting for MBot thread to join")
        mbot_thread_handle.join()
    if lidar_thread_handle.is_alive():
        logging.info("Waiting for Lidar thread to join")
        lidar_thread_handle.join()

    # Close the socket
    lidar_client.close()
    mbot_client.close()
    pose_command_connection.close()
    pose_command_server.close()

    # Quit pygame
    pygame.quit()

if __name__ == "__main__":
    main()