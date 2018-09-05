#!/usr/bin/env python

# Roland Arsenault
# Center for Coastal and Ocean Mapping
# University of New Hampshire
# Copyright 2017, All rights reserved.

import math
import time
import threading

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_msgs.msg import Float32
from rosgraph_msgs.msg import Clock
from geographic_msgs.msg import GeoPointStamped
from marine_msgs.msg import NavEulerStamped

import asv_sim.dynamics
import asv_sim.environment
import asv_sim.coastal_surveyor
import asv_sim.cw4

class AsvSim:
    def __init__(self,model="cw4"):
        self.throttle = 0.0
        self.rudder = 0.0
        self.last_command_timestamp = None
        
        self.environment = asv_sim.environment.Environment()
        
        if model == "cw4":
            self.dynamics = asv_sim.dynamics.Dynamics(asv_sim.cw4.cw4,self.environment)
        elif model == "coastal_surveyor":
            self.dynamics = asv_sim.dynamics.Dynamics(asv_sim.coastal_surveyor.coastal_surveyor,self.environment)
        
        self.wallclock_time_step = 0.05
        self.sim_time = rospy.Time.from_sec(time.time())
        self.time_factor = 1.0


    def run(self):
        rospy.init_node('asv_sim')
        self.position_publisher = rospy.Publisher('/position', GeoPointStamped, queue_size = 5)
        self.heading_publisher = rospy.Publisher('/heading', NavEulerStamped, queue_size = 5)
        self.clock_publisher = rospy.Publisher('/clock', Clock, queue_size = 5)
        self.clock_factor_subscriber = rospy.Subscriber('/clock_factor', Float64, self.clock_factor_callback)
        self.cmd_vel_subscriber = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Timer(rospy.Duration.from_sec(0.05),self.update)
        clock_timer = threading.Timer(self.wallclock_time_step,self.update_clock)
        clock_timer.start()
        rospy.spin()
        
    def clock_factor_callback(self, data):
        self.time_factor = data.data
    
        
    def update_clock(self):
        self.sim_time += rospy.Duration.from_sec(self.wallclock_time_step*self.time_factor)
        c = Clock(self.sim_time)
        self.clock_publisher.publish(c)
        if not rospy.is_shutdown():
            clock_timer = threading.Timer(self.wallclock_time_step,self.update_clock)
            clock_timer.start()

    def cmd_vel_callback(self, data):
        self.throttle = max(min(data.linear.x/self.dynamics.model['max_speed'],1.0),-1.0)
        self.rudder = max(min(data.angular.z,1.0),-1.0)
        self.last_command_timestamp = self.sim_time
        
    def update(self, event):
        if self.last_command_timestamp is None or event.current_real - self.last_command_timestamp > rospy.Duration.from_sec(0.5*self.time_factor):
            self.throttle = 0.0
            self.rudder = 0.0
        
        self.dynamics.update(self.throttle,self.rudder,event.current_real)
        
        gps = GeoPointStamped()
        gps.header.stamp = self.dynamics.last_update
        
        gps.position.latitude = math.degrees(self.dynamics.latitude)
        gps.position.longitude = math.degrees(self.dynamics.longitude)
        
        self.position_publisher.publish(gps)

        
        #p.basic_position.cog = self.dynamics.cog
        #p.basic_position.sog = self.dynamics.sog
        #p.header.stamp = self.dynamics.last_update
        
        h = NavEulerStamped()
        h.header.stamp = self.dynamics.last_update
        h.orientation.heading = math.degrees(self.dynamics.heading)
        self.heading_publisher.publish(h)

if __name__ == '__main__':
    try:
        sim = AsvSim()
        sim.run()
    except rospy.ROSInterruptException:
        pass
