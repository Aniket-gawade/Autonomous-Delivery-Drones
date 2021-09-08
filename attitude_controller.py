#!/usr/bin/env python


from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu ,NavSatFix
from std_msgs.msg import Float32
import rospy
import time
import tf

PI=3.1415926535897


class Edrone():
    
    def __init__(self):
        rospy.init_node('attitude_controller')  

        
        self.drone_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]

        self.drone_orientation_euler = [0.0, 0.0, 0.0]

        self.setpoint_cmd = [0.0, 0.0, 0.0, 0.0 ,0.0 ,0.0 ,0.0 ,0.0]  #ROLL ,PITCH ,YAW ,ALTITUDE & aux1,aux2,aux3,aux4

        self.prev_error = [0,0,0]

        self.error = [0,0,0]

        self.Iterm = [0,0,0]

        self.max_value = 1024

        self.min_value = 0

        self.out_roll = 0
        self.out_pitch = 0
        self.out_yaw = 0
        self.out_thrust = 0
        
        
        self.pwm_cmd = prop_speed()
        self.pwm_cmd.prop1 = 0
        self.pwm_cmd.prop2 = 0
        self.pwm_cmd.prop3 = 0
        self.pwm_cmd.prop4 = 0


        # self.alt_error =  z_error()  # to plot
        # self.alt_error.z_error = 0

        # self.zero =  zero_error() #to plot 
        # self.zero.zero_error = 0

        self.Kp = [0,0,0] #ROLL PITCH YAW gains
        self.Ki = [0,0,0]
        self.Kd = [0,0,0]
       
        self.sample_time = 0.016 

        
        self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=50)
        # self.z_error_pub = rospy.Publisher('z_error', z_error, queue_size=50) # for plotting
        # self.zero_error_pub = rospy.Publisher('zero_error', zero_error, queue_size=50) 


        
        rospy.Subscriber('/drone_command', edrone_cmd, self.drone_command_callback)
        rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)
        # rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        # rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)   #for tuning the controller
        # rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
        # rospy.Subscriber('/pid_tuning_yaw', PidTune, self.yaw_set_pid)
        # rospy.Subscriber('/pid_tuning_altitude', PidTune, self.altitude_set_pid)
        

       


    def imu_callback(self, msg):   

        self.drone_orientation_quaternion[0] = msg.orientation.x  #from the IMU sensor
        self.drone_orientation_quaternion[1] = msg.orientation.y
        self.drone_orientation_quaternion[2] = msg.orientation.z
        self.drone_orientation_quaternion[3] = msg.orientation.w


    def drone_command_callback(self, msg):

        self.setpoint_cmd[0] = msg.rcRoll   #drone_command topic which will receive messages from postion controller
        self.setpoint_cmd[1] = msg.rcPitch
        self.setpoint_cmd[2] = msg.rcYaw
        self.setpoint_cmd[3] = msg.rcThrottle
        self.setpoint_cmd[4] = msg.aux1
        self.setpoint_cmd[5] = msg.aux2
        self.setpoint_cmd[6] = msg.aux3
        self.setpoint_cmd[7] = msg.aux4
        
    # def roll_set_pid(self, roll):   #for tuning the controller    

    #     self.Kp[0] = roll.Kp   
    #     self.Ki[0] = roll.Ki * 0.008
    #     self.Kd[0] = roll.Kd 

    # def pitch_set_pid(self, pitch):

        # self.Kp[1] = pitch.Kp * 0.06  
        # self.Ki[1] = pitch.Ki * 0.008
        # self.Kd[1] = pitch.Kd * 0.3

    # def yaw_set_pid(self, yaw):

        # self.Kp[2] = yaw.Kp * 0.06  
        # self.Ki[2] = yaw.Ki * 0.008
        # self.Kd[2] = yaw.Kd * 0.3

    

        
    def pid(self):

        global pitch,roll,yaw,desired_roll,desired_pitch,desired_yaw
        
        (self.drone_orientation_euler[0], self.drone_orientation_euler[1], self.drone_orientation_euler[2]) = tf.transformations.euler_from_quaternion([self.drone_orientation_quaternion[0], self.drone_orientation_quaternion[1], self.drone_orientation_quaternion[2], self.drone_orientation_quaternion[3]])

        desired_roll = self.setpoint_cmd[0] * 0.02 - 30 
        desired_pitch = self.setpoint_cmd[1] * 0.02 - 30 
        desired_yaw = self.setpoint_cmd[2] * 0.02 - 30

        
        roll = (self.drone_orientation_euler[1]*180)/PI    #converting current angles in degrees for better accuracy 
        pitch = (self.drone_orientation_euler[0]*180)/PI
        yaw = (self.drone_orientation_euler[2]*180)/PI 

        # print roll 
        # print pitch
        # print yaw 
        


        self.error[0] = desired_roll - roll
        self.error[1] = desired_pitch - pitch
        self.error[2] = desired_yaw - yaw
        

        # print self.error[0]
        # print self.error[1]
        # print self.error[2]
         

        # self.alt_error.z_error = self.error[0]  #for plotting only
        # self.z_error_pub.publish(self.alt_error)

        # self.zero.zero_error = 0 #for plotting only
        # self.zero_error_pub.publish(self.zero)


        # final gains of roll ,pitch ,yaw 


        self.Kp[2] = 12 #yaw gains
        self.Ki[2] = 0
        self.Kd[2] = 15

        self.Kp[1] = 7 #pitch gains
        self.Ki[1] = 0
        self.Kd[1] = 100

        self.Kp[0] = 7 #roll gains
        self.Ki[0] = 0
        self.Kd[0] = 100



        # 3 PID's for Roll,Pitch, Yaw 


        self.out_roll = self.Kp[0]*self.error[0] + self.Iterm[0] + self.Kd[0]*(self.error[0] - self.prev_error[0])  

        self.out_pitch = self.Kp[1]*self.error[1] + self.Iterm[1] + self.Kd[1]*(self.error[1] - self.prev_error[1]) 

        self.out_yaw = self.Kp[2]*self.error[2] + self.Iterm[2] + self.Kd[2]*(self.error[2] - self.prev_error[2]) 

        self.out_thrust = self.setpoint_cmd[3] - 1500   
   
        
        #Converting to PWM value range and propellers speed are controlled when required for error correction

        # for roll ,pitch,yaw and thrust motion



        self.pwm_cmd.prop1 = 500 + self.out_thrust - self.out_yaw + self.out_pitch - self.out_roll
        self.pwm_cmd.prop2 = 500 + self.out_thrust + self.out_yaw - self.out_pitch - self.out_roll
        self.pwm_cmd.prop3 = 500 + self.out_thrust - self.out_yaw - self.out_pitch + self.out_roll
        self.pwm_cmd.prop4 = 500 + self.out_thrust + self.out_yaw + self.out_pitch + self.out_roll


        #limiting conditions

        if self.pwm_cmd.prop1 > self.max_value:
            self.pwm_cmd.prop1 = self.max_value

        if self.pwm_cmd.prop1 < self.min_value:
            self.pwm_cmd.prop1 = self.min_value

        if self.pwm_cmd.prop2 > self.max_value:
            self.pwm_cmd.prop2 = self.max_value

        if self.pwm_cmd.prop2 < self.min_value:
            self.pwm_cmd.prop2 = self.min_value

        if self.pwm_cmd.prop3 > self.max_value:
            self.pwm_cmd.prop3 = self.max_value

        if self.pwm_cmd.prop3 < self.min_value:
            self.pwm_cmd.prop3 = self.min_value

        if self.pwm_cmd.prop4 > self.max_value:
            self.pwm_cmd.prop4 = self.max_value

        if self.pwm_cmd.prop4 < self.min_value:
            self.pwm_cmd.prop4 = self.min_value




        self.pwm_pub.publish(self.pwm_cmd) #publishing pwm values 

        self.prev_error[0] = self.error[0]  #for the D-term 
        self.prev_error[1] = self.error[1]
        self.prev_error[2] = self.error[2]
       


        self.Iterm[0] = (self.Iterm[0] + self.error[0])*self.Ki[0]  #for the I-term 
        self.Iterm[1] = (self.Iterm[1] + self.error[1])*self.Ki[1]
        self.Iterm[2] = (self.Iterm[2] + self.error[2])*self.Ki[2]
        



if __name__ == '__main__':
    try:
        e_drone = Edrone()
        r = rospy.Rate(1/e_drone.sample_time)  
        while not rospy.is_shutdown():
            e_drone.pid()
            r.sleep()
    except rospy.ROSInterruptException:
        pass
