#!/usr/bin/env python

# Roland Arsenault
# Center for Coastal and Ocean Mapping
# University of New Hampshire
# Copyright 2017, All rights reserved.

from builtins import object
import math
import time
import threading

import rospy
import tf.transformations

from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import Float64
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from rosgraph_msgs.msg import Clock

from asv_sim.srv import SetPose

import asv_sim.dynamics
import asv_sim.environment
import asv_sim.coastal_surveyor
import asv_sim.cw4

from dynamic_reconfigure.server import Server
from asv_sim.cfg import asv_simConfig

class AsvSim(object):
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

        self.mru_frame = rospy.get_param('~mru_frame', 'mru')

    def run(self):
        rospy.init_node('asv_sim')
        self.position_publisher = rospy.Publisher('position', NavSatFix, queue_size = 5)
        self.orientation_publisher = rospy.Publisher('orientation', Imu, queue_size = 5)
        self.velocity_publisher = rospy.Publisher('velocity', TwistWithCovarianceStamped, queue_size = 5)
        self.clock_publisher = rospy.Publisher('/clock', Clock, queue_size = 5)
        
        self.diag_publishers = {}
        self.have_commands_pub = rospy.Publisher('asv_sim/have_commands', Bool, queue_size = 5)
        
        self.clock_factor_subscriber = rospy.Subscriber('/clock_factor', Float64, self.clock_factor_callback)
        
        self.throttle_subscriber = rospy.Subscriber('throttle', Float32, self.throttle_callback)
        self.rudder_subscriber = rospy.Subscriber('rudder', Float32, self.rudder_callback)
        
        self.reset_subscriber = rospy.Subscriber('sim_reset', Bool, self.reset_callback)

        self.set_pose_service = rospy.Service('set_pose', SetPose, self.set_pose, buff_size=5)
        
        srv = Server(asv_simConfig, self.reconfigure_callback)
        
        rospy.Timer(rospy.Duration.from_sec(0.05),self.update)
        clock_timer = threading.Timer(self.wallclock_time_step,self.update_clock)
        clock_timer.start()
        rospy.spin()
        
    def clock_factor_callback(self, data):
        self.time_factor = data.data
    
    def reset_callback(self, data):
        self.dynamics.reset()

    def set_pose(self, req):
        self.dynamics.set(req.point.position.latitude, req.point.position.longitude, req.nav.orientation.heading)
        return True

    def update_clock(self):
        self.sim_time += rospy.Duration.from_sec(self.wallclock_time_step*self.time_factor)
        c = Clock(self.sim_time)
        self.clock_publisher.publish(c)
        if not rospy.is_shutdown():
            clock_timer = threading.Timer(self.wallclock_time_step,self.update_clock)
            clock_timer.start()

    def throttle_callback(self, data):
        self.throttle = max(min(data.data,1.0),-1.0)
        self.last_command_timestamp = self.sim_time
        
    def rudder_callback(self, data):
        self.rudder = max(min(data.data,1.0),-1.0)

    def reconfigure_callback(self, config, level):
        self.environment.current['speed'] = config['current_speed']
        self.environment.current['direction'] = config['current_direction']
        
        for item in ('max_rpm','max_power','idle_rpm','max_rpm_change_rate','max_speed','mass','max_rudder_angle','rudder_distance','rudder_coefficient'):
            self.dynamics.model[item] = config['dynamics_'+item]
        for item in ('thrust','rudder','drag','current_speed','current_direction'):
            self.dynamics.jitters[item] = config['jitter_'+item]
        self.dynamics.update_coefficients()
        return config
        
    def update(self, event):
        if self.last_command_timestamp is None or event.current_real - self.last_command_timestamp > rospy.Duration.from_sec(0.5*self.time_factor):
            self.throttle = 0.0
            self.rudder = 0.0
            self.have_commands_pub.publish(False)
        else:
            self.have_commands_pub.publish(True)
        
        diag = self.dynamics.update(self.throttle,self.rudder,event.current_real)
        
        nsf = NavSatFix()
        nsf.header.stamp = self.dynamics.last_update
        nsf.header.frame_id = self.mru_frame
        nsf.latitude = math.degrees(self.dynamics.latitude)
        nsf.longitude = math.degrees(self.dynamics.longitude)
        self.position_publisher.publish(nsf)

        imu = Imu()                  
        imu.header.stamp = self.dynamics.last_update
        imu.header.frame_id = self.mru_frame
        yaw = math.radians(90.0)-self.dynamics.heading
        q = tf.transformations.quaternion_from_euler(yaw, 0, 0, 'rzyx')
        imu.orientation.x = q[0]
        imu.orientation.y = q[1]
        imu.orientation.z = q[2]
        imu.orientation.w = q[3]
        imu.angular_velocity.z = -self.dynamics.yaw_rate
        imu.linear_acceleration.x = self.dynamics.a
        self.orientation_publisher.publish(imu)

        twcs = TwistWithCovarianceStamped()
        twcs.header.stamp = self.dynamics.last_update
        twcs.header.frame_id = self.mru_frame
        
        sin_yaw = math.sin(yaw)
        cos_yaw = math.cos(yaw)
        
        twcs.twist.twist.linear.x = sin_yaw*self.dynamics.sog
        twcs.twist.twist.linear.y = cos_yaw*self.dynamics.sog
        self.velocity_publisher.publish(twcs)
        
        for key,value in diag.items():
            if not key in self.diag_publishers:
                self.diag_publishers[key] = rospy.Publisher('asv_sim/diagnostics/'+key, Float64, queue_size = 5)
            self.diag_publishers[key].publish(value)
        

if __name__ == '__main__':
    try:
        sim = AsvSim()
        sim.run()
    except rospy.ROSInterruptException:
        pass
