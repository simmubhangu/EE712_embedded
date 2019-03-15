#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Float32
import sys, select, termios, tty

class get_postion():
    """docstring for get_postion"""
    def __init__(self):
        # self.arg = arg
        rospy.init_node('get_position')
        self.settings = termios.tcgetattr(sys.stdin)
        rospy.Subscriber("lwheel", Float32, self.get_l_wheel)
        rospy.Subscriber("rwheel",Float32,self.get_r_wheel)
        self.r_wheel_enc =0
        self.l_wheel_enc =0
        self.t_delta = rospy.Duration(1.0/20)
        self.t_next = rospy.Time.now() + self.t_delta
        self.then = rospy.Time.now()
        self.wheel_dia = 6.5    #cm
        self.base_width = 24.5  #cm
        self.enc_left = 1
        self.x = 0                  # position in xy plane 
        self.y = 0
        self.th = 0
        self.rate =20

    def get_l_wheel(self,data):
         self.l_wheel_enc = data.data
        
    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key        
    def get_r_wheel(self,data):
        self.r_wheel_enc =  data.data
    
    def update(self):
    #############################################################################
        now = rospy.Time.now()
        if now > self.t_next:
            elapsed = now - self.then
            self.then = now
            elapsed = elapsed.to_sec()
            

            # calculate odometry
            if self.enc_left == None:
                d_left = 0
                d_right = 0
            else:
                d_left = self.l_wheel_enc * self.wheel_dia
                d_right = self.r_wheel_enc * self.wheel_dia
                self.enc_left =d_left
       
           
            # distance traveled is the average of the two wheels 
            d = ( d_left + d_right ) / 2
            # this approximation works (in radians) for small angles
            th = ( d_right - d_left ) / self.base_width
            # calculate velocities
            self.dx = d / elapsed
            self.dr = th / elapsed
           

             
            if (d != 0):
                # calculate distance traveled in x and y
                x = math.cos( th ) * d
                y = -math.sin( th ) * d
                # calculate the final position of the robot
                self.x = self.x + ( math.cos( self.th ) * x - math.sin( self.th ) * y )
                self.y = self.y + ( math.sin( self.th ) * x + math.cos( self.th ) * y )
                print  "x-axis = " + str(self.x) , "y-axis = " + str(self.y)
            if( th != 0):
                self.th = self.th + th
                print "theta = " + str(self.th)  

    def print_data(self):
        while True:
            key = self.getKey()
            # self.update()
            if (key == '\x03'):
                break  
    
    def spin(self):
    #############################################################################
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            # self.update()
            r.sleep()                                                            

if __name__ == '__main__':
    while not rospy.is_shutdown():
        postion = get_postion()
        postion.spin()
        # rospy.spin()    

