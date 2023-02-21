#!/usr/bin/python3

import rospy
import roslaunch
import tf
import time
import numpy
import matplotlib.pyplot as plt
from math import atan, atan2, degrees, radians, sqrt, tanh, tan, sin, cos, pi
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from datetime import date, datetime

DIST_PRECISION      = 0.002
DIST_PRECISION2     = atan(2/1000)
BURGER_STEP_LIN_VEL = 0.15
TANH_PARAM_LIN_VEL  = 8
RECORD_ODOM_POSITION = True
RECORD_OPTITRACK_POSITION = True

global data
data = []
global data2
data2 = []

now = datetime.now()
if RECORD_ODOM_POSITION == True:
    recorded_odom_position = open('cw' + '.txt', "w")
    recorded_odom_position2 = open('ccw' + '.txt', "w")

if RECORD_OPTITRACK_POSITION == True:
    recorded_optitrack_position = open('opt_cw' + '.txt', "w")
    recorded_optitrack_position2 = open('opt_ccw' + '.txt', "w")

#--------------------------------POTREBNE FUNCKIJE--------------------------------------#
def Odom_pomocni():
     rospy.loginfo("Starting pomocni...")
     package = 'pozicija'
     executable = 'odom_pomocni.py'
     node = roslaunch.core.Node(package, executable)
     launch = roslaunch.scriptapi.ROSLaunch()
     launch.start()
     process = launch.launch(node)
     print (process.is_alive())
     return process

def Odom_optitrack():
    rospy.loginfo("Starting optitrack...")
    package = 'pozicija'
    executable = 'odom_optitrack.py'
    node = roslaunch.core.Node(package, executable)
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    process2 = launch.launch(node)
    print (process2.is_alive())
    return process2

def getCurrentPosition( listener_current):
    (current_pose_translation, current_pose_rotation) = listener_current.lookupTransform("odom_pomocni", "base_link", rospy.Time(0))
    alfa, beta, current_theta_rad = tf.transformations.euler_from_quaternion(current_pose_rotation)
    currentX = current_pose_translation[0]
    currentY = current_pose_translation[1]
    del listener_current
    return currentX, currentY, current_theta_rad
      
def getOptitrackPosition(listener_optitrack):
    (optitrack_pose_translation, optitrack_pose_rotation) = listener_optitrack.lookupTransform("odom_optitrack", "robot", rospy.Time(0))
    alfa, beta, optitrack_theta_rad = tf.transformations.euler_from_quaternion(optitrack_pose_rotation)
    optitrackX = optitrack_pose_translation[0]
    optitrackY = optitrack_pose_translation[1]
    del listener_optitrack
    return optitrackX, optitrackY, optitrack_theta_rad

def getABSDistance(current_x, current_y, goal_x, goal_y):
    distance = sqrt(pow((goal_x - current_x), 2) + pow((goal_y - current_y), 2)) 
    return distance

def stopAllMotors(seconds):
    last = rospy.get_rostime()
    now = rospy.get_rostime()
    cmd_vel_stop_pub_pose_control = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    cmd_vel_stop = Twist()
    cmd_vel_stop.linear.x  = 0.0
    cmd_vel_stop.linear.y  = 0.0
    cmd_vel_stop.linear.z  = 0.0
    cmd_vel_stop.angular.x = 0.0
    cmd_vel_stop.angular.y = 0.0
    cmd_vel_stop.angular.z = 0.0
    cmd_vel_stop_pub_pose_control.publish(cmd_vel_stop)
    while(now.secs - last.secs < seconds):
        cmd_vel_stop_pub_pose_control.publish(cmd_vel_stop)
        now = rospy.get_rostime()

#--------------------------------PLOTANJE--------------------------------------#
def Plotting():
    Eb, Ed = Calibration()
    
    #plot 1
    plt.subplot(1, 2, 1) 
    plt.title("Clockwise")
    x1=0
    x2=1000
    y1=0
    y2=1000
    x = [x1, x2, x2, x1, x1]
    y = [y1, y1, y2, y2, y1]

    with open('opt_cw.txt', 'r') as f:
        lines = f.readlines()
        X = []
        Y = []
        F1 = []
        G1 = []
        for i in lines:
            linija = i.split()
            X.append(1000*float(linija[0]))
            Y.append(-1000*float(linija[1]))
            if int(linija[3]) == 2:
                F1.append(1000*float(linija[0]))
                G1.append(-1000*float(linija[1]))
            if int(linija[3]) == 1:
                F1.append(1000*float(linija[0]))
                G1.append(-1000*float(linija[1]))

    # with open('cw.txt', 'r') as f:
    #     lines2 = f.readlines()
    #     Xr = []
    #     Yr = []
    #     for i in lines2:
    #         linija2 = i.split()
    #         Xr.append(1000*float(linija2[0]))
    #         Yr.append(-1000*float(linija2[1]))

    #plt.plot(Yr, Xr, 'b', linewidth = 2, label = 'Robot')       
    plt.plot(Y, X, 'b', linewidth = 1.5, label = 'Optitrack')   
    plt.plot(G1, F1, 'w', linewidth = 1.5) 
    plt.plot(x, y, 'r--', linewidth = 2.0, label = 'Referenca')
    plt.scatter(0, 0, c = 'k', linewidth = 1.5, marker = 'x', s = 200)
    plt.xlabel('y [mm]')
    plt.ylabel('x [mm]')
    plt.legend(loc = 'upper right')
    plt.axis('equal')
    props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
    textstr = '\n'.join((
    'Ed = %.3f' % (Ed, ),
    'Eb = %.3f' % (Eb, )))
    plt.text(0, -270, textstr, fontsize=12, bbox=props)
    

    #plot 2
    plt.subplot(1, 2, 2) 
    plt.title("Counter-clockwise")
    x1=0
    x2=-1*1000
    y1=0
    y2=1000
    x = [x1, x2, x2, x1, x1]
    y = [y1, y1, y2, y2, y1]

    with open('opt_ccw.txt', 'r') as f:
        lines = f.readlines()
        X = []
        Y = []
        F1 = []
        G1 = []
        for i in lines:
            linija = i.split()
            X.append(1000*float(linija[0]))
            Y.append(-1000*float(linija[1]))
            if int(linija[3]) == 2:
                F1.append(1000*float(linija[0]))
                G1.append(-1000*float(linija[1]))
            if int(linija[3]) == 1:
                F1.append(1000*float(linija[0]))
                G1.append(-1000*float(linija[1]))

    with open('ccw.txt', 'r') as f:
        lines2 = f.readlines()
        Xr = []
        Yr = []
        for i in lines2:
            linija2 = i.split()
            Xr.append(1000*float(linija2[0]))
            Yr.append(-1000*float(linija2[1]))
    
    #plt.plot(Yr, Xr, 'y', linewidth = 1, label = 'Robot')
    plt.plot(Y, X, 'b', linewidth = 1.5, label = 'Optitrack')        
    plt.plot(G1, F1, 'w', linewidth = 1.5)         
    plt.plot(x, y, 'r--', linewidth = 2.0, label = 'Referenca')
    plt.scatter(0, 0, c = 'k', linewidth = 1.5, marker = 'x', s = 200)
    plt.xlabel('y [mm]')
    plt.ylabel('x [mm]')
    plt.legend(loc = 'upper right')
    plt.axis('equal')
    props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
    textstr = '\n'.join((
    'Ed = %.3f' % (Ed, ),
    'Eb = %.3f' % (Eb, )))
    plt.text(-1000, -80, textstr, fontsize=12, bbox=props)

    plt.show()
#------------------------------------------------------------------------------#

#--------------------------------KALIBRACIJA-----------------------------------#
def Calibration():
    #POCETNA POZICIJA JE UVIJEK: x0 = 0.0 i y0 = 0.0
    with open('opt_cw.txt', 'r') as f:
        podatci = f.readlines()
        Xn = []
        Yn = []
        for i in podatci:
            red = i.split()
            if int(red[3]) == 2:
                Xn.append(float(red[0]))
                Yn.append(-1*float(red[1]))
    
    f.close()
   
    with open('opt_ccw.txt', 'r') as f:
        podatci2 = f.readlines()
        Xm = []
        Ym = []
        for i in podatci2:
            red2 = i.split()
            if int(red2[3]) == 2:
                Xm.append(float(red2[0]))
                Ym.append(-1*float(red2[1]))
          
    f.close()
   
    Xcg_cw = 0
    Ycg_cw = 0
    Xcg_ccw = 0
    Ycg_ccw = 0
    for i in range(len(Xn)):
        Xcg_cw += 0 - Xn[i]
        Ycg_cw += 0 - Yn[i]
    for i in range(len(Xm)):
        Xcg_ccw += 0 - Xm[i]
        Ycg_ccw += 0 - Ym[i]
    
    #koordinate tezista klustera u smjeru kazaljke na satu
    Xcg_cw = Xcg_cw/len(Xn)
    Ycg_cw = Ycg_cw/len(Yn)
   
    #koordinate tezista klustera u smjeru suprotno od kazaljke na satu
    Xcg_ccw = Xcg_ccw/len(Xm)
    Ycg_ccw = Ycg_ccw/len(Ym)
    
    b = 0.165
    L = 1 #duljina stranice kvadrata
    betaX = ((Xcg_cw - Xcg_ccw)/-4*L)
    alphaX = ((Xcg_cw + Xcg_ccw)/-4*L)*(180/pi)
    R = (L/2)/sin(betaX/2)#polumjer skretanja
    Eb = (90/(90-alphaX)) #omjer stvarnog i nazivnog razmaka izmedju kotaca
    Ed = (R + (b/2))/(R - (b/2)) #omjer promjera desnog i lijevog kotaca
    return Eb, Ed
#----------------------------------------------------------------------------------------#

#------------------------------------KRETANJE--------------------------------------------#
def straight(dx):
    #translacija
    currentX, currentY, current_theta_rad = getCurrentPosition(listener_current)
    optitrackX, optitrackY, optitrack_theta_rad = getOptitrackPosition(listener_optitrack)
    pocetniX = currentX
    pocetniY = currentY
    frame_distance = getABSDistance(currentX, currentY, pocetniX, pocetniY)
    cmd_vel_pub_pose_control = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    cmd_vel = Twist()

    last = rospy.get_rostime()
    now = rospy.get_rostime()
    while(now.secs - last.secs < 1.5):
        cmd_vel_pub_pose_control.publish(cmd_vel)
        now = rospy.get_rostime()
        rate.sleep()
    
    while(frame_distance < dx - DIST_PRECISION):
        if not rospy.is_shutdown():

            currentX, currentY, current_theta_rad = getCurrentPosition(listener_current)
            optitrackX, optitrackY, optitrack_theta_rad = getOptitrackPosition(listener_optitrack)
            frame_distance = getABSDistance(currentX, currentY, pocetniX, pocetniY)
            print(frame_distance)
            cmd_vel.linear.x  = tanh(TANH_PARAM_LIN_VEL * (dx-frame_distance)) * BURGER_STEP_LIN_VEL
            cmd_vel.linear.y  = 0.0
            cmd_vel.linear.z  = 0.0
            cmd_vel.angular.x = 0.0
            cmd_vel.angular.y = 0.0
            cmd_vel.angular.z = 0.0

            if(frame_distance > dx - DIST_PRECISION - 0.02):
                cmd_vel.linear.x  = 0.003
                cmd_vel.linear.y  = 0.0
                cmd_vel.linear.z  = 0.0
                cmd_vel.angular.x = 0.0
                cmd_vel.angular.y = 0.0
                cmd_vel.angular.z = 0.0

            cmd_vel_pub_pose_control.publish(cmd_vel)
            data.append([currentX, currentY, current_theta_rad, 0])
            data2.append([optitrackX, optitrackY, optitrack_theta_rad, 0])

        else:
            stopAllMotors(2.0)
            print("USER INTERRUPTED")
            break
        rate.sleep()
        

    if dx - frame_distance < DIST_PRECISION:
        print(frame_distance)
        stopAllMotors(2) 
        print("REACHED!")

    stopAllMotors(2) 

def turn(a, b, c):
    #rotacija
    
    currentX, currentY, current_theta_rad = getCurrentPosition(listener_current)
    optitrackX, optitrackY, optitrack_theta_rad = getOptitrackPosition(listener_optitrack)
    cmd_vel_pub_pose_control = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    cmd_vel = Twist()
    kut = numpy.deg2rad(a)

    last = rospy.get_rostime()
    now = rospy.get_rostime()
    while(now.secs - last.secs < 1.5):
        cmd_vel_pub_pose_control.publish(cmd_vel)
        now = rospy.get_rostime()
        rate.sleep()
     
    while abs(kut - current_theta_rad) > DIST_PRECISION2:
        if not rospy.is_shutdown():

            print(abs(kut - current_theta_rad))
            cmd_vel.linear.x  = 0.0
            cmd_vel.linear.y  = 0.0
            cmd_vel.linear.z  = 0.0
            cmd_vel.angular.x = 0.0
            cmd_vel.angular.y = 0.0
            cmd_vel.angular.z  = b*(abs(kut - current_theta_rad))
            
            if abs(current_theta_rad - kut) < DIST_PRECISION + 0.02:
                cmd_vel.linear.x  = 0.0
                cmd_vel.linear.y  = 0.0
                cmd_vel.linear.z  = 0.0
                cmd_vel.angular.x = 0.0
                cmd_vel.angular.y = 0.0
                cmd_vel.angular.z = c

            cmd_vel_pub_pose_control.publish(cmd_vel)
            currentX, currentY, current_theta_rad = getCurrentPosition(listener_current)
            optitrackX, optitrackY, optitrack_theta_rad = getOptitrackPosition(listener_optitrack)

            data.append([currentX, currentY, current_theta_rad, 0])
            data2.append([optitrackX, optitrackY, optitrack_theta_rad, 0])

        else:
            stopAllMotors(2.0)
            print("USER INTERRUPTED")
            break
        rate.sleep()  

    if abs(kut - current_theta_rad) < DIST_PRECISION:
        print(abs(kut - current_theta_rad))
        stopAllMotors(2)     
        print("TURNED!")
        
    #stopAllMotors(2)     
#----------------------------------------------------------------------------------------#

#------------------------------------PROGRAM---------------------------------------------#

if __name__ == '__main__':
    rospy.init_node('kvadrat2')
    rate = rospy.Rate(50)
    for i in range(1):
        process = Odom_pomocni()    
        listener_current = tf.TransformListener()
        listener_current.waitForTransform("odom_pomocni", "base_link", rospy.Time(), rospy.Duration(10.0))
        process2 = Odom_optitrack()    
        listener_optitrack = tf.TransformListener() 
        listener_optitrack.waitForTransform("odom_optitrack", "robot", rospy.Time(), rospy.Duration(10.0))  
        currentX, currentY, current_theta_rad = getCurrentPosition(listener_current)
        optitrackX, optitrackY, optitrack_theta_rad = getOptitrackPosition(listener_optitrack)
        data.append([currentX, currentY, current_theta_rad, 1])
        data2.append([optitrackX, optitrackY, optitrack_theta_rad, 1])
        straight(1)
        turn(-90, -0.1, -0.003)
        straight(1)
        turn(-180, -0.1, -0.003)
        straight(1)
        turn(90, -0.1, -0.003)
        straight(1)
        turn(0, -0.1, -0.003)
        currentX, currentY, current_theta_rad = getCurrentPosition(listener_current)
        optitrackX, optitrackY, optitrack_theta_rad = getOptitrackPosition(listener_optitrack)
        data.append([currentX, currentY, current_theta_rad, 2])
        data2.append([optitrackX, optitrackY, optitrack_theta_rad, 2])
        process.stop()
        process2.stop()

    for i in data:
        recorded_odom_position.writelines(str(i[0]) + '   ' + str(i[1]) + '   ' + str(i[2]) + '   ' + str(i[3]) + '\n')
    for i in data2:
        recorded_optitrack_position.writelines(str(i[0]) + '   ' + str(i[1]) + '   ' + str(i[2]) + '   ' + str(i[3]) + '\n')

    recorded_odom_position.close()
    recorded_optitrack_position.close()
    data = []
    data2 = []

    for i in range(5):
        process = Odom_pomocni()
        listener_current = tf.TransformListener()
        listener_current.waitForTransform("odom_pomocni", "base_link", rospy.Time(), rospy.Duration(10.0))
        process2 = Odom_optitrack()
        listener_optitrack = tf.TransformListener() 
        listener_optitrack.waitForTransform("odom_optitrack", "robotino", rospy.Time(), rospy.Duration(10.0))  
        currentX, currentY, current_theta_rad = getCurrentPosition(listener_current)
        optitrackX, optitrackY, optitrack_theta_rad = getOptitrackPosition(listener_optitrack)
        data.append([currentX, currentY, current_theta_rad, 1])
        data2.append([optitrackX, optitrackY, optitrack_theta_rad, 1])
        straight(1)
        turn(90, 0.3, 0.003)
        straight(1)
        turn(180, 0.3, 0.003)
        straight(1)
        turn(-90, 0.3, 0.003)
        straight(1)
        turn(0, 0.3, 0.003)
        currentX, currentY, current_theta_rad = getCurrentPosition(listener_current)
        optitrackX, optitrackY, optitrack_theta_rad = getOptitrackPosition(listener_optitrack)
        data.append([currentX, currentY, current_theta_rad, 2])
        data2.append([optitrackX, optitrackY, optitrack_theta_rad, 2])
        process.stop()  
        process2.stop()

    for i in data:
        recorded_odom_position2.writelines(str(i[0]) + '   ' + str(i[1]) + '   ' + str(i[2]) + '   ' + str(i[3]) + '\n')
    for i in data2:
        recorded_optitrack_position2.writelines(str(i[0]) + '   ' + str(i[1]) + '   ' + str(i[2]) + '   ' + str(i[3]) + '\n')

    recorded_odom_position2.close()
    recorded_optitrack_position2.close()

    Calibration()
    Plotting()

    rate.sleep()
#--------------------------------KRAJ--------------------------------------#