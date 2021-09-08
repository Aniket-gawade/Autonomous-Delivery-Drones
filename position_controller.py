#!/usr/bin/env python


from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import NavSatFix, LaserScan
from std_msgs.msg import *
from vitarana_drone.srv import *
import gripper_service
import rosservice
import rospy
import time
import tf
import math


class Edrone():
    
    def __init__(self):

        rospy.init_node('position_controller')  

        
        self.setpoint_cmd = [0.0,0.0,0.0]

        #starting position : lat: 18.9992411380, long: 71.9998195496, alt: 16.66    

        # Building 1: lat: 18.9990965928, long: 72.0000664814, alt: 10.75
        # Building 2: lat: 18.9990965925, long: 71.9999050292, alt: 22.2
        # Building 3: lat: 18.9993675932, long: 72.0000569892, alt: 10.7

 

        self.setpoint_1 = [18.9990965928,72.0000664814,30] 

        self.setpoint_2 = [19.0007046575,71.9998955286,25]

        self.setpoint_3 = [19.0007024000,71.9998928000,21.94]

        self.setpoint_4 = [19.0006500000,71.9998928000,25]

        self.prev_error = [0,0,0]

        self.error = [0,0,0]

        self.Iterm = [0,0,0]

        #output roll pitch altitude limiting conditions

        self.max_values = [1550,1550,1550]  

        self.min_values = [1450,1450,1450] 



        #gains for the 3 PID's (lat,long,alt)

        self.Kp = [0,0,0]  
        self.Ki = [0,0,0]  
        self.Kd = [0,0,0]

        self.out_roll_angle = 0
        self.out_pitch_angle = 0
        self.out_thrust = 0

        self.drone_latitude = 0
        self.drone_longitude = 0
        self.drone_altitude = 0

    #current x and y in meters
        self.x = 0
        self.y = 0

    #setpoint x ,y and z in meters
        self.setpoint_x = 0 
        self.setpoint_y = 0
        self.setpoint_z = 0

    #destination setpoint from QR code
        self.destination_latitude = 0
        self.destination_longitude = 0
        self.destination_altitude  = 0

    #Detecting the obstacle using LIDAR sensor on drone
        self.obstacle_detect = [0.0,0.0,0.0,0.0]
        self.obst_angle_inc = 0

    #landing marker co-ordinates from cascade function
        self.landing_x = 0
        self.landing_y = 0

        self.sample_time = 0.016 

        self.pos_cmd = edrone_cmd()
        self.pos_cmd.rcRoll = 0
        self.pos_cmd.rcPitch = 0
        self.pos_cmd.rcYaw = 0
        self.pos_cmd.rcThrottle = 0

        self.condition = True
        self.path_planner = False
        self.count = 0
        self.avoid = False


        
        self.pos_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=50)

        
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        # rospy.Subscriber('/pid_tuning_altitude', PidTune, self.altitude_set_pid)
        rospy.Subscriber('/dest_set',setpoint, self.dest_set)
        rospy.Subscriber('/edrone/gripper_check',String,self.gripper_check_callback)
        rospy.Subscriber('/edrone/range_finder_top',LaserScan,self.obstacle_detect_callback)
        rospy.Subscriber('/edrone/marker_data',MarkerData,self.landing_marker_callback)


    def gps_callback(self, msg):

        self.drone_latitude = msg.latitude #gps values
        self.drone_longitude = msg.longitude
        self.drone_altitude = msg.altitude

        # print self.drone_latitude
        # print self.drone_longitude
        # print self.drone_altitude

    # def altitude_set_pid(self, altitude):

    #     self.Kp[0] = altitude.Kp 
    #     self.Ki[0] = altitude.Ki*0.01 
    #     self.Kd[0] = altitude.Kd*100

    def dest_set(self, msg):

        self.destination_latitude = msg.lat #qr_detect values
        self.destination_longitude = msg.long
        self.destination_altitude = msg.alt


    def gripper_check_callback(self, msg):
        if msg.data == 'True' and self.count==0:
            print ('package is pickable')
            self.count = self.count+1

    def activate_gripper_client(self):
        rospy.wait_for_service('/edrone/activate_gripper')
        try:
            activate_gripper = rospy.ServiceProxy('/edrone/activate_gripper', Gripper)
            resp1 = activate_gripper(self.condition)
            return resp1.result
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def landing_marker_callback(self, msg):

        self.landing_x = msg.err_x_m 
        self.landing_y = msg.err_y_m 


    def obstacle_detect_callback(self ,msg):

        self.obstacle_detect[0] = msg.ranges[0]  #front side obstacle 
        self.obstacle_detect[1] = msg.ranges[1]  #right side obstacle
        self.obstacle_detect[2] = msg.ranges[2]  #back side obstacle
        self.obstacle_detect[3] = msg.ranges[3]  #left side obstacle
        self.obst_angle_inc = msg.angle_increment


        # Dynamically changing the setpoints using a ladder

        if ((18.9992411300<self.drone_latitude<18.9992411400) and (71.9998195400<self.drone_longitude<71.9998195500) and (16.6<self.drone_altitude<16.7)):
            self.setpoint_cmd = self.setpoint_1
            rospy.loginfo("ascending")
        

        # elif ((19.0009245000<self.drone_latitude<19.0009250000) and (71.9998310000<self.drone_longitude<72.9998320000) and (24.78<self.drone_altitude<25)):
        #     self.setpoint_cmd = self.setpoint_2
        #     rospy.loginfo("flight")

        # elif ((19.0007047000<self.drone_latitude<19.0007049000) and (71.9998952000<self.drone_longitude<71.9998954000) and (25<self.drone_altitude<25.2)):
        #     self.setpoint_cmd = self.setpoint_3
        #     rospy.loginfo("descending")

        # elif ((19.0007046500<self.drone_latitude<19.0007048500) and (71.9998952000<self.drone_longitude<71.9998954000) and (22.16<self.drone_altitude<22.18)):
        #     self.setpoint_cmd = self.setpoint_4
        #     rospy.loginfo("package picked up")
            
        # elif ((19.0006400000<self.drone_latitude<19.0006500000) and (71.9998900000<self.drone_longitude<71.9998999000) and (25<self.drone_altitude<25.2)):
        #     self.path_planner = True

        # elif ((18.9999995500<self.drone_latitude<19.0000090000) and (71.999995251<self.drone_longitude<72.0000050000) and (25.0<self.drone_altitude<25.2)):
        #     rospy.loginfo("descending")
        #     self.path_planner = False
        #     self.setpoint_cmd[2] = self.destination_altitude

        # elif ((18.9999995500<self.drone_latitude<19.0000090000) and (71.999995251<self.drone_longitude<72.0000050000) and (8.4<self.drone_altitude<8.7)):
        #     self.condition = False
        #     rospy.loginfo("package successfully delivered")


        if self.path_planner:
            if (self.obstacle_detect[3]>8):
                rospy.loginfo("going to goal")
                self.setpoint_x = 110692.0702932625 * (self.destination_latitude  - 19) 
                self.setpoint_y = -105292.0089353767 * (self.destination_longitude  - 72)  
                self.setpoint_z = 25  
            

            #Potential Field Algorithim for obstacle avoidance
            elif  (self.obstacle_detect[3]<=8 and self.obstacle_detect[3]>0.3):
                avoidance_vector_x=0
                avoidance_vector_y=0
                self.avoid = False

                for i in range(1,len(self.obstacle_detect)):
                    d0 = 8
                    k = 0.1
                    if ((self.obstacle_detect[3]<d0) and (self.obstacle_detect[3] > 0.0599999986589)): 
                        rospy.loginfo("avoiding obstacle")
                        self.avoid = True
                        x = math.cos(self.obst_angle_inc*i)
                        y = math.sin(self.obst_angle_inc*i)        
                        u = -0.5*k*pow((1/self.obstacle_detect[3]) - (1/d0) , 2)

                        avoidance_vector_x = avoidance_vector_x + x*u
                        avoidance_vector_y = avoidance_vector_y + y*u



                if self.avoid:
                    if math.sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)) > 8:
                        avoidance_vector_x = 8*(avoidance_vector_x/math.sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)))
                        avoidance_vector_y = 8*(avoidance_vector_y/math.sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)))



                    self.setpoint_x = avoidance_vector_x + self.x
                    self.setpoint_y = avoidance_vector_y + self.y   


 
    def lat_to_x(self):
        self.x = 110692.0702932625 * (self.drone_latitude - 19) #current latitude in meters

    def long_to_y(self):
        self.y = -105292.0089353767 * (self.drone_longitude  - 72)  #current longitude in meters

    def setpoint_to_meters(self):
        if (not(self.avoid)):
            self.setpoint_x = 110692.0702932625 * (self.setpoint_cmd[0]  - 19) #conversion used by setpoints in ladder 
            self.setpoint_y = -105292.0089353767 * (self.setpoint_cmd[1]  - 72) 
        self.setpoint_z = self.setpoint_cmd[2]


        
    def pid(self):


        self.error[0] = self.setpoint_x - self.x
        self.error[1] = -1*(self.setpoint_y - self.y)
        self.error[2] = self.setpoint_z - self.drone_altitude


        # final gains for lat,long,alt

        self.Kp[0] = 10 #latitude gains
        self.Ki[0] = 0
        self.Kd[0] = 6000

        self.Kp[1] = 10  #longitude gains
        self.Ki[1] = 0
        self.Kd[1] = 6000

        self.Kp[2] = 45   #altitude gains
        self.Ki[2] = 0
        self.Kd[2] = 5000 

        # 3 PID's for latitude,longitude,altitude

        self.out_roll_angle = 1500 + self.Kp[0]*self.error[0] + self.Iterm[0] + self.Kd[0]*(self.error[0] - self.prev_error[0])  

        self.out_pitch_angle = 1500 + self.Kp[1]*self.error[1] + self.Iterm[1] + self.Kd[1]*(self.error[1] - self.prev_error[1]) 

        self.out_thrust =  1500 + self.Kp[2]*self.error[2] + self.Iterm[2] + self.Kd[2]*(self.error[2] - self.prev_error[2])

        
        
        #Converting to angle range and publishing roll,pitch,yaw setpoints for attitude controller  

        self.pos_cmd.rcRoll = self.out_roll_angle 
        self.pos_cmd.rcPitch = self.out_pitch_angle
        self.pos_cmd.rcYaw = 1500                    #no rotation in z -axis required for task
        self.pos_cmd.rcThrottle = self.out_thrust  

        
        
        

        if self.pos_cmd.rcRoll > self.max_values[0]:
            self.pos_cmd.rcRoll = self.max_values[0]

        if self.pos_cmd.rcRoll < self.min_values[0]:
            self.pos_cmd.rcRoll = self.min_values[0]

        if self.pos_cmd.rcPitch > self.max_values[1]:
            self.pos_cmd.rcPitch = self.max_values[1]

        if self.pos_cmd.rcPitch < self.min_values[1]:
            self.pos_cmd.rcPitch = self.min_values[1]

        if self.pos_cmd.rcThrottle > self.max_values[2]:
            self.pos_cmd.rcThrottle = self.max_values[2]

        if self.pos_cmd.rcThrottle < self.min_values[2]:
            self.pos_cmd.rcThrottle = self.min_values[2]

        
        self.pos_pub.publish(self.pos_cmd)


        self.prev_error[0] = self.error[0]   #for the D-term
        self.prev_error[1] = self.error[1]
        self.prev_error[2] =  self.error[2]
        


        self.Iterm[0] = (self.Iterm[0] + self.error[0])*self.Ki[0]   #for the I-term
        self.Iterm[1] = (self.Iterm[1] + self.error[1])*self.Ki[1]
        self.Iterm[2] = (self.Iterm[2] + self.error[2])*self.Ki[2]
        



if __name__ == '__main__':
    try:
        e_drone = Edrone()
        r = rospy.Rate(1/e_drone.sample_time)  
        while not rospy.is_shutdown():
            e_drone.pid()
            e_drone.activate_gripper_client()
            e_drone.lat_to_x()
            e_drone.long_to_y()
            e_drone.setpoint_to_meters()
            r.sleep()
    except rospy.ROSInterruptException:
        pass
