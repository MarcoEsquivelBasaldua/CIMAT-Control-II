from controller import *
from controller import Camera
from controller import Keyboard
import mavic2proHelper
import cv2
from simple_pid import PID
import csv
import struct
import numpy as np
from numpy import savetxt

params = dict()
with open("../params.csv", "r") as f:
	lines = csv.reader(f)
	for line in lines:
		params[line[0]] = line[1]
		
		
TIME_STEP = int(params["QUADCOPTER_TIME_STEP"])
TAKEOFF_THRESHOLD_VELOCITY = int(params["TAKEOFF_THRESHOLD_VELOCITY"])
M_PI = 3.1415926535897932384626433
MIN_MATCH_COUNT = 4
I = np.eye(3)


def angle_diff(target, current):
    angle = np.absolute(target - current)
    sign = 0.0
    
    if 2*M_PI - angle < angle:
        angle =  2*M_PI - angle
        
        if target < current:
            sign = 1.0
        else:
            sign = -1.0
    else:
        if target > current:
          sign = 1.0
        else:
          sign = -1.0
          
    return -sign*angle
    


robot = Robot()

[frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor] = mavic2proHelper.getMotorAll(robot)

timestep = int(robot.getBasicTimeStep())
mavic2proMotors = mavic2proHelper.getMotorAll(robot)
mavic2proHelper.initialiseMotors(robot, 0)
mavic2proHelper.motorsSpeed(robot, TAKEOFF_THRESHOLD_VELOCITY, TAKEOFF_THRESHOLD_VELOCITY, TAKEOFF_THRESHOLD_VELOCITY, TAKEOFF_THRESHOLD_VELOCITY)

front_left_led = LED("front left led")
front_right_led = LED("front right led")
gps = GPS("gps")
gps.enable(TIME_STEP)
imu = InertialUnit("inertial unit")
imu.enable(TIME_STEP)
compass = Compass("compass")
compass.enable(TIME_STEP)
gyro = Gyro("gyro")
gyro.enable(TIME_STEP)
camera = Camera("camera")
camera.enable(TIME_STEP)
keyboard = Keyboard()
keyboard.enable(TIME_STEP)

targetYaw = 0.0

pitchPID = PID(float(params["pitch_Kp"]), float(params["pitch_Ki"]), float(params["pitch_Kd"]), setpoint=0.0)
rollPID = PID(float(params["roll_Kp"]), float(params["roll_Ki"]), float(params["roll_Kd"]), setpoint=0.0)
throttlePID = PID(float(params["throttle_Kp"]), float(params["throttle_Ki"]), float(params["throttle_Kd"]), setpoint=1)
yawPID = PID(float(params["yaw_Kp"]), float(params["yaw_Ki"]), float(params["yaw_Kd"]), setpoint=float(targetYaw))


targetX, targetY, target_altitude = -0.4, -2.0, 0.3

if targetYaw < 0.0:
    targetYaw = 2*M_PI + targetYaw

last_error_yaw = 0.0
d_error_yaw = 0.0
kp_yaw = 2.0
kd_yaw = 2.0

# Target image
target_image = cv2.imread('target_img.png')

# Initiate SIFT detector
sift = cv2.SIFT_create()
kp1, des1 = sift.detectAndCompute(target_image,None)


# Camera params and Intrinsic matrix
ImageSizeX = camera.getWidth()
ImageSizeY = camera.getHeight()

CameraHFOV = camera.getFov()
CameraVFOV = 2 * np.arctan(np.tan(CameraHFOV * 0.5) * (ImageSizeY / ImageSizeX))

cx = ImageSizeX/2
cy = ImageSizeY/2
fx = ImageSizeX /(2*np.tan(CameraHFOV*0.5))
fy = ImageSizeY /(2*np.tan(CameraVFOV*0.5))

K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])
K_inv = np.linalg.inv(K)

l_v = 0.8
l_omega = 2.2

startControl = False
last_time = 0.0
it = 0
epsilon = 0.02


# To get speeds as averaes
dim = 10
vels_x = np.zeros(dim)
vels_y = np.zeros(dim)
vels_z = np.zeros(dim)
vels_yaw = np.zeros(dim)
j = 0
flag = True

# List to draw graphs at the end
drone_pos = []
drone_vel = []
drone_time = []
t_start = 0.0


startSim = True
while (robot.step(timestep) != -1):
    time = robot.getTime()
    dt = time - last_time
    last_time = time
    
    it += 1
    
    led_state = int(robot.getTime()) % 2
    front_left_led.set(led_state)
    front_right_led.set(int(not(led_state)))
    
    # Get current sensor measurements
    roll = imu.getRollPitchYaw()[0] + M_PI / 2.0
    pitch = imu.getRollPitchYaw()[1]
    
    yaw = imu.getRollPitchYaw()[2] + M_PI
    if yaw < 0.0:
        yaw = 2*M_PI + yaw
    
    roll_acceleration = gyro.getValues()[0]
    pitch_acceleration = gyro.getValues()[1]
    xGPS = gps.getValues()[2]
    yGPS = -gps.getValues()[0]
    zGPS = gps.getValues()[1]
    ###################################
    
    
    
    # Get control from keyboard
    key = keyboard.getKey()
    while key > 0:
        if key == Keyboard.UP:
            targetY += 0.01
        elif key == Keyboard.DOWN:
            targetY -= 0.01
        elif key == Keyboard.LEFT:
            targetX += 0.01
        elif key == Keyboard.RIGHT:
            targetX -= 0.01
        elif key == Keyboard.SHIFT + Keyboard.LEFT:
            targetYaw += 0.01
            if targetYaw > 2*M_PI:
                targetYaw = 0.0
        elif key == Keyboard.SHIFT + Keyboard.RIGHT:
            targetYaw -= 0.01
            if targetYaw < 0.0:
                targetYaw = 2*M_PI
        elif key == Keyboard.SHIFT + Keyboard.UP:
            target_altitude += 0.01
        elif key == Keyboard.SHIFT + Keyboard.DOWN:
            target_altitude -= 0.01
        elif key == Keyboard.PAGEUP:
            camera.saveImage('init.png',0)
        elif key == Keyboard.HOME:
            startControl = True
            t_start = time
        elif key == Keyboard.END:
            startSim = False
        
        key = keyboard.getKey()
    
    
    if startControl and startSim:
        # Get current image
        cameraData = camera.getImage()
        current_image = np.frombuffer(cameraData, np.uint8).reshape((ImageSizeY, ImageSizeX, 4))
        
        kp2, des2 = sift.detectAndCompute(current_image,None)
        
        FLANN_INDEX_KDTREE = 1
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks = 500)
        
        # Flann Matcher
        flann = cv2.FlannBasedMatcher(index_params, search_params)
        matches = flann.knnMatch(des1,des2,k=2)
        
        # Apply ratio test
        good = []
        for m,n in matches:
            if m.distance < 0.7*n.distance:
                good.append(m)
                
        if len(good)>MIN_MATCH_COUNT:
            src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
            dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
            
            H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 3.0)
            
            
            matchesMask = mask.ravel().tolist()
            
            h,w,d = target_image.shape
            pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
            dst = cv2.perspectiveTransform(pts,H)
            
            current_image = cv2.polylines(current_image,[np.int32(dst)],True,255,3, cv2.LINE_AA)
            
            #############################
            # Euclidean Homography
            H = np.dot(np.dot(K_inv, H), K)
            #H = H/np.abs(H[2, 2])
            
            # Compute error
            m = np.array([kp1[0].pt[0]/ImageSizeX, kp1[0].pt[1]/ImageSizeY, 1.0])
            #m = np.array([kp1[0].pt[0], kp1[0].pt[1], 1.0])
            errorv = np.dot((H - I), m)
            M = H - H.T
            errorw =  np.array([M[2, 1], M[0, 2], M[1, 0]])
            
            # Compute Velocity
            vel_lin = -np.dot(l_v * I, errorv)
            vel_ang = -np.dot(l_omega * I, errorw)
            print(vel_lin)
            print(vel_ang)
            ############################
            
            """
            #####
            # Descomponer la homografia 
            num, Rs, Ts, Ns  = cv2.decomposeHomographyMat(H, K)
            T = Ts[0]
            R = np.zeros((3,3))
            n = 0
            
            # Elegir el vector de traslacion y la matriz de rotacion
            dis = np.inf
            i = 0
            for t in Ns:
                norm = np.linalg.norm(t - np.array([[0.0],[0.0],[1.0]]))
                if norm < dis:
                    dis = norm
                    n = i 
                    T = Ts[n]
                    
                i += 1
            
            R = Rs[n]
            
            # Encuentra el vector-angulo con Rodrigues
            rvec,_ = cv2.Rodrigues(R)
            
            # Veelocidades (control)
            vel_lin = -l_v * np.dot(R.T, T)
            vel_ang = -l_omega * rvec
            #####
            """
            
            vels_x[j] = vel_lin[0]
            vels_y[j] = vel_lin[2]
            vels_z[j] = vel_lin[1]
            vels_yaw[j] = vel_ang[1]
            
            
            if it % dim == 0:
                j = 0
                
                Tc1 = np.array([[xGPS], [yGPS], [zGPS], [yaw]])
                Tc = np.array([[xGPS], [yGPS], [zGPS]])
                n_Tc = np.linalg.norm(Tc)
                if n_Tc < epsilon:
                    startSim = False
                    
                    # Save data
                    file_time = open("time.txt","w+")
                    savetxt(file_time, drone_time, fmt ='%.2f')
                    file_time.close
                    
                    
                    file_pos = open("pos.txt","w+")
                    savetxt(file_pos, np.array(drone_pos).reshape(-1,4), fmt ='%.3f')
                    file_pos.close

                    file_vel = open("vel.txt","w+")
                    savetxt(file_vel, np.array(drone_vel).reshape(-1,4), fmt ='%.3f')
                    file_vel.close
                    
                    targetX += 0.0
                    targetY += 0.0
                    target_altitude += 0.0
                    
                    targetYaw += 0.0
            
                
                # Define new targets with Euler method
                mean_x = np.mean(vels_x)
                mean_y = np.mean(vels_y)
                mean_z = np.mean(vels_z)
                mean_yaw = np.mean(vels_yaw)
                
                targetX += mean_x * dt
                targetY += mean_y * dt
                target_altitude += mean_z * dt
                    
                targetYaw += mean_yaw * dt
                if targetYaw > 2*M_PI:
                    targetYaw = 0.0
                elif targetYaw < 0.0:
                    targetYaw = 2*M_PI
                
                vels = np.array([[mean_x], [mean_y], [mean_z], [mean_yaw]])
                drone_pos.append(Tc1)
                drone_vel.append(vels)
                drone_time.append(time - t_start)
        
                vels_x *= 0.0
                vels_y *= 0.0
                vels_z *= 0.0
                vels_yaw *= 0.0
            else:
                j += 1
            
        else:
            print( "Not enough matches are found - {}/{}".format(len(good), MIN_MATCH_COUNT) )
            matchesMask = None
        
        draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                       singlePointColor = None,
                       matchesMask = matchesMask, # draw only inliers
                       flags = 2)
                   
    
        img_matches = cv2.drawMatches(target_image, kp1, current_image, kp2, good, None, **draw_params)
        
        #cv2.imshow('Current image', current_image)
        #cv2.imshow('Target image', target_image)
        cv2.imshow('Matches (Target image - Current image)', img_matches)
        cv2.waitKey(TIME_STEP)
        
    
    
    
    vertical_input = throttlePID(zGPS)
    
    error_yaw = angle_diff(targetYaw, yaw)
    d_error_yaw = error_yaw - last_error_yaw
    last_error_yaw = error_yaw
    yaw_input = kp_yaw*error_yaw  + kd_yaw*d_error_yaw
    
    rollPID.setpoint = targetX
    pitchPID.setpoint = targetY
    throttlePID.setpoint = target_altitude
    
    roll_input = float(params["k_roll_p"]) * roll + roll_acceleration + rollPID(xGPS)
    pitch_input = float(params["k_pitch_p"]) * pitch - pitch_acceleration + pitchPID(yGPS)
    
    front_left_motor_input = float(params["k_vertical_thrust"]) + vertical_input - roll_input - pitch_input + yaw_input
    front_right_motor_input = float(params["k_vertical_thrust"]) + vertical_input + roll_input - pitch_input - yaw_input
    rear_left_motor_input = float(params["k_vertical_thrust"]) + vertical_input - roll_input + pitch_input - yaw_input
    rear_right_motor_input = float(params["k_vertical_thrust"]) + vertical_input + roll_input + pitch_input + yaw_input
    
    mavic2proHelper.motorsSpeed(robot, front_left_motor_input, -front_right_motor_input, -rear_left_motor_input, rear_right_motor_input)
    #mavic2proHelper.motorsSpeed(robot, 0, 0, 0, 0)
    
    print('------------------------')
    print('Desided coordinates')
    print('x: ', targetX)
    print('y: ', targetY)
    print('z: ', target_altitude)
    print('yaw: ', targetYaw)
    
    print('Current coordinates')
    print('x: ', xGPS)
    print('y: ', yGPS)
    print('z: ', zGPS)
    print('yaw: ', yaw)
    
    print('time: ', time)
    
