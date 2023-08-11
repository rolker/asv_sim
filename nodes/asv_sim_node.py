#!/usr/bin/env python3

# Roland Arsenault
# Center for Coastal and Ocean Mapping
# University of New Hampshire
# Copyright 2017, All rights reserved.

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

from dynamic_reconfigure.server import Server
from asv_sim.cfg import environmentConfig
from asv_sim.cfg import dynamicsConfig
from asv_sim.cfg import jitterConfig

class Platform:
    def __init__(self, name, model, environment, sim, namespace, parameters):
        self.name = name
        self.sim = sim
        self.throttle = 0.0
        self.rudder = 0.0
        self.last_command_timestamp = None

        start = {}
        if 'start_lat' in parameters:
            start['lat'] = parameters['start_lat']
        if 'start_lon' in parameters:
            start['lon'] = parameters['start_lon']
        if 'start_heading' in parameters:
            start['heading'] = parameters['start_heading']

        self.dynamics = asv_sim.dynamics.Dynamics(model, environment, start)

        self.namespace = namespace

        self.mru_frame = 'mru'
        if 'mru_frame' in parameters:
            self.mru_frame = parameters['mru_frame']

    def run(self):
        self.position_publisher = rospy.Publisher(self.namespace+'/position', NavSatFix, queue_size = 5)
        self.orientation_publisher = rospy.Publisher(self.namespace+'/orientation', Imu, queue_size = 5)
        self.velocity_publisher = rospy.Publisher(self.namespace+'/velocity', TwistWithCovarianceStamped, queue_size = 5)

        self.diag_publishers = {}
        self.have_commands_pub = rospy.Publisher('~'+self.name+'/have_commands', Bool, queue_size = 5)

        self.throttle_subscriber = rospy.Subscriber(self.namespace+'/throttle', Float32, self.throttle_callback)
        self.rudder_subscriber = rospy.Subscriber(self.namespace+'/rudder', Float32, self.rudder_callback)

        self.reset_subscriber = rospy.Subscriber('~'+self.name+'/sim_reset', Bool, self.reset_callback)

        self.set_pose_service = rospy.Service('~'+self.name+'/set_pose', SetPose, self.set_pose, buff_size=5)
        
        self.dynamics_srv = Server(dynamicsConfig, self.dynamics_reconfigure_callback, self.name+'/dynamics')
        self.jitter_srv = Server(jitterConfig, self.jitter_reconfigure_callback, self.name+'/jitter')

    def reset_callback(self, data):
        self.dynamics.reset()

    def set_pose(self, req):
        self.dynamics.set(req.point.position.latitude, req.point.position.longitude, req.nav.orientation.heading)
        return True

    def throttle_callback(self, data):
        self.throttle = max(min(data.data,1.0),-1.0)
        if self.sim.use_sim_time:
            self.last_command_timestamp = self.sim.sim_time
        else:
            self.last_command_timestamp = rospy.Time.now()
        
    def rudder_callback(self, data):
        self.rudder = max(min(data.data,1.0),-1.0)

    def dynamics_reconfigure_callback(self, config, level):
        for item in ('max_rpm','max_power','idle_rpm','max_rpm_change_rate','max_speed','mass','max_rudder_angle','rudder_distance','rudder_coefficient'):
            self.dynamics.model[item] = config[item]
        self.dynamics.update_coefficients()
        return config

    def jitter_reconfigure_callback(self, config, level):
        for item in ('thrust','rudder','drag','current_speed','current_direction'):
            self.dynamics.jitters[item] = config[item]
        self.dynamics.update_coefficients()
        return config

    def update(self, event):
        rospy.logdebug(event)
        if self.last_command_timestamp is None or event.current_real - self.last_command_timestamp > rospy.Duration.from_sec(0.5*sim.time_factor):
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
        

        course_angle = math.radians(90.0)-self.dynamics.cog
        sin_cog = math.sin(course_angle)
        cos_cog = math.cos(course_angle)
        
        twcs.twist.twist.linear.x = cos_cog*self.dynamics.sog
        twcs.twist.twist.linear.y = sin_cog*self.dynamics.sog
        self.velocity_publisher.publish(twcs)
        
        for key,value in diag.items():
            if not key in self.diag_publishers:
                self.diag_publishers[key] = rospy.Publisher('~'+self.name+'/diagnostics/'+key, Float64, queue_size = 5)
            self.diag_publishers[key].publish(value)


class AsvSim(object):
    def __init__(self,model="cw4"):
        rospy.init_node('asv_sim')

        self.use_sim_time = rospy.get_param("/use_sim_time", False)

        models = rospy.get_param('~models')

        platforms = rospy.get_param("~platforms")
        
        self.environment = asv_sim.environment.Environment()

        self.platforms = []

        for p in platforms:
            config = platforms[p]
            model = config['model']
            namespace = '/'+p
            if 'namespace' in config:
                namespace = config['namespace']
            rospy.loginfo(p+": model: "+model+", ns: " + namespace)

            self.platforms.append(Platform(p, models[model], self.environment, self, namespace, config))
        
        if self.use_sim_time:
            self.wallclock_time_step = 0.05
            self.sim_time = rospy.Time.from_sec(time.time())
        self.time_factor = 1.0


    def run(self):

        if self.use_sim_time:
            self.clock_publisher = rospy.Publisher('/clock', Clock, queue_size = 5)
            self.clock_factor_subscriber = rospy.Subscriber('/clock_factor', Float64, self.clock_factor_callback)
        
        self.reset_subscriber = rospy.Subscriber('~sim_reset', Bool, self.reset_callback)

        srv = Server(environmentConfig, self.reconfigure_callback)

        for p in self.platforms:
            p.run()
        
        rospy.Timer(rospy.Duration.from_sec(0.05),self.update)
        if self.use_sim_time:
            clock_timer = threading.Timer(self.wallclock_time_step, self.update_clock)
            clock_timer.start()
        rospy.spin()
        
    def clock_factor_callback(self, data):
        self.time_factor = data.data
    
    def reset_callback(self, data):
        for p in self.platforms:
            p.reset_callback(data)

    def update_clock(self):
        self.sim_time += rospy.Duration.from_sec(self.wallclock_time_step*self.time_factor)
        c = Clock(self.sim_time)
        self.clock_publisher.publish(c)
        if not rospy.is_shutdown():
            clock_timer = threading.Timer(self.wallclock_time_step,self.update_clock)
            clock_timer.start()


    def reconfigure_callback(self, config, level):
        self.environment.current['speed'] = config['current_speed']
        self.environment.current['direction'] = config['current_direction']
        
        return config
        
    def update(self, event):
        for p in self.platforms:
            p.update(event)

if __name__ == '__main__':
    try:
        sim = AsvSim()
        sim.run()
    except rospy.ROSInterruptException:
        pass
