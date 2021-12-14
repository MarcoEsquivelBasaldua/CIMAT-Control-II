from controller import *
from controller import Camera
from controller import Keyboard
import mavic2proHelper
import cv2
from simple_pid import PID
import csv
import struct
import numpy as np
import matplotlib.pyplot as plt

params = dict()
with open("../params.csv", "r") as f:
	lines = csv.reader(f)
	for line in lines:
		params[line[0]] = line[1]
		
		
TIME_STEP = int(params["QUADCOPTER_TIME_STEP"])
TAKEOFF_THRESHOLD_VELOCITY = int(params["TAKEOFF_THRESHOLD_VELOCITY"])
M_PI = 3.1415926535897932384626433
MIN_MATCH_COUNT = 4

x_offset = 3.5
y_offset = -23.5
z_offset = -4
yaw_offset = -6.1 + 2*M_PI

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
    
def unitize(x1, y1):
    n = np.sqrt(x1**2 + y1**2)
    x = x1/n
    y = y1/n
    
    return x, y

def homog2Rt(H):
    U, s, V = np.linalg.svd(H, full_matrices = False)
    
    s1 = s[0] / s[1]
    s3 = s[2] / s[1]
    
    zeta = s1 - s3
    a1 = np.sqrt(1 - s3**2)
    b1 = np.sqrt(s1**2 - 1)
    
    a, b = unitize(a1, b1)
    c, d = unitize(1+s1*s3, a1*b1)
    e, f = unitize(-b/s1, -a/s3)
    
    v1 = V[:,0]
    v3 = V[:,2]
    n1 = b*v1 - a*v3
    n2 = b*v1 + a*v3
    
    R1 = U.dot(np.array([[c, 0.0, d], [0.0, 1.0, 0.0], [-d, 0.0, c]])).dot(V.T)
    R2 = U.dot(np.array([[c, 0.0, -d], [0.0, 1.0, 0.0], [d, 0.0, c]])).dot(V.T)
    
    t1 = -R1.dot(e*v1 + f*v3)
    t2 = -R2.dot(e*v1 - f*v3)
    
    if n1[2] < 0.0:
        t1 = -R1.dot(t1)
        n1 = -n1
    
    if n2[2] < 0.0:
        t2 = -R2.dot(t2)
        n2 = -n2
        
    
    return t1, R1, t2, R2



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

targetX, targetY, target_altitude = 0.0, 0.0, 1.0
targetX += x_offset
targetY += y_offset
target_altitude += z_offset

targetYaw += yaw_offset
if targetYaw < 0.0:
    targetYaw = 2*M_PI + targetYaw

last_error_yaw = 0.0
d_error_yaw = 0.0
kp_yaw = 2.0
kd_yaw = 2.0

#camera.getImage()
#camera.saveImage('init.png',0)
#Camera.getImage(camera)
#Camera.saveImage(camera,"initial_image.png",0)
#cv2.startWindowThread()
#cv2.namedWindow('preview')
target_image = cv2.imread('target_img32.png')

# Initiate SIFT detector
sift = cv2.SIFT_create()
kp1, des1 = sift.detectAndCompute(target_image,None)

# Camera params and Intrinsic matrix
#camera.setFocalDistance(0.01)
ImageSizeX = camera.getWidth()
ImageSizeY = camera.getHeight()

CameraHFOV = camera.getFov()
CameraVFOV = 2 * np.arctan(np.tan(CameraHFOV * 0.5) * (ImageSizeY / ImageSizeX))

cx = ImageSizeX/2
cy = ImageSizeY/2
fx = ImageSizeX /(np.tan(CameraHFOV))
fy = ImageSizeY /(np.tan(CameraVFOV))
f = ImageSizeX/(1/np.tan(CameraHFOV*0.5))

K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])
K_inv = np.linalg.inv(K)

while (robot.step(timestep) != -1):
    time = robot.getTime()
    led_state = int(robot.getTime()) % 2
    front_left_led.set(led_state)
    front_right_led.set(int(not(led_state)))
    
    # Get current sensor measurements
    roll = imu.getRollPitchYaw()[0] + M_PI / 2.0
    pitch = imu.getRollPitchYaw()[1]
    
    yaw = imu.getRollPitchYaw()[2] + M_PI + yaw_offset
    if yaw < 0.0:
        yaw = 2*M_PI + yaw
    
    roll_acceleration = gyro.getValues()[0]
    pitch_acceleration = gyro.getValues()[1]
    xGPS = gps.getValues()[2] + x_offset
    yGPS = -gps.getValues()[0] + y_offset
    zGPS = gps.getValues()[1] + z_offset
    ###################################
    
    # Get and display current image
    
    cameraData = camera.getImage()
    current_image = np.frombuffer(cameraData, np.uint8).reshape((ImageSizeY, ImageSizeX, 4))
    #current_image = cv2.cvtColor(current_image, cv2.COLOR_BGR2GRAY)
    
    kp2, des2 = sift.detectAndCompute(current_image,None)
    
    FLANN_INDEX_KDTREE = 1
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks = 50)
    
    # Flann Matcher
    flann = cv2.FlannBasedMatcher(index_params, search_params)
    matches = flann.knnMatch(des1,des2,k=2)
    
    # Apply ratio test
    good = []
    for m,n in matches:
        if m.distance < 0.7*n.distance:
            good.append(m)
    
    if len(good)>MIN_MATCH_COUNT:
        src_pts = np.int32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
        dst_pts = np.int32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
        
        H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
        
        matchesMask = mask.ravel().tolist()
        
        h,w,d = target_image.shape
        pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
        dst = cv2.perspectiveTransform(pts,H)
        
        current_image = cv2.polylines(current_image,[np.int32(dst)],True,255,3, cv2.LINE_AA)
        
        
        #H_ = K_inv.dot(H)
        H_ = np.dot(np.dot(K_inv, H),K)
        
        
        t1, R1, t2, R2 = homog2Rt(H_)
        num, Rs, Ts, Ns  = cv2.decomposeHomographyMat(H, K)
        
        
        dis = np.inf
        i = 0
        for t in Ns:
            norm = np.linalg.norm(t - np.array([[0.0],[0.0],[1.0]]))
            if norm < dis:
                dis = norm
                T = t
                n = i    
            i += 1
        R = Rs[n]
        # Homography decomposition
        
        H = H.T
        h1 = H[0]
        h2 = H[1]
        h3 = H[2]
        L = 1 / np.linalg.norm(np.dot(K_inv, h1))
        r1 = L * np.dot(K_inv, h1)
        r2 = L * np.dot(K_inv, h2)
        r3 = np.cross(r1, r2)
        T = L * (K_inv @ h3.reshape(3, 1))
        R = np.array([[r1], [r2], [r3]])
        R = np.reshape(R, (3, 3))
        
    else:
        print( "Not enough matches are found - {}/{}".format(len(good), MIN_MATCH_COUNT) )
        matchesMask = None
    
    #img_matches = cv2.drawMatchesKnn(current_image, kpCurrent, target_image, kpTarget, good, None, flags = cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                   singlePointColor = None,
                   matchesMask = matchesMask, # draw only inliers
                   flags = 2)
    img_matches = cv2.drawMatches(target_image, kp1, current_image, kp2, good, None, **draw_params)
    
    cv2.imshow('Current image', current_image)
    cv2.imshow('Target image', target_image)
    cv2.imshow('Matches', img_matches)
    cv2.waitKey(TIME_STEP)
    
    
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
        elif key == Keyboard.END:
            camera.saveImage('target.png',0)
        
        key = keyboard.getKey()
    
    
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
    
    #mavic2proHelper.motorsSpeed(robot, front_left_motor_input, -front_right_motor_input, -rear_left_motor_input, rear_right_motor_input)
    mavic2proHelper.motorsSpeed(robot, 0, 0, 0, 0)
    
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
    print(t1)
    print(t2)
    print(Ts)
    print(T)