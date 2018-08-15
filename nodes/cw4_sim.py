#!/usr/bin/env python

# Roland Arsenault
# Center for Coastal and Ocean Mapping
# University of New Hampshire
# Copyright 2017, All rights reserved.

import math
import time
import threading

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from rosgraph_msgs.msg import Clock
from asv_srvs.srv import VehicleState
from asv_srvs.srv import VehicleStateRequest
from asv_srvs.srv import VehicleStateResponse
from asv_srvs.srv import PilotControl
from asv_srvs.srv import PilotControlRequest
from asv_srvs.srv import PilotControlResponse
from asv_msgs.msg import HeadingHold
from asv_msgs.msg import BasicPositionStamped
from asv_msgs.msg import HeadingStamped
from asv_msgs.msg import VehicleStatus

import asv_sim.dynamics
import asv_sim.environment
import asv_sim.coastal_surveyor
import asv_sim.cw4


class AsvSim:
    def __init__(self,model="cw4"):
        self.vehicle_state = VehicleStateRequest.VP_STATE_PAUSE
        self.pilot_control = False
        self.inhibit = True
        
        self.throttle = 0.0
        self.rudder = 0.0
        
        self.envoronment = asv_sim.environment.Environment()
        
        if model == "cw4":
            self.dynamics = asv_sim.dynamics.Dynamics(asv_sim.cw4.cw4,self.envoronment)
        elif model == "coastal_surveyor":
            self.dynamics = asv_sim.dynamics.Dynamics(asv_sim.coastal_surveyor.coastal_surveyor,self.envoronment)
        
        self.last_heading_hold_timestamp = None
        self.wallclock_time_step = 0.05
        self.sim_time = rospy.Time.from_sec(time.time())
        self.time_factor = 1.0


    def run(self):
        rospy.init_node('asv_sim')
        self.state_service = rospy.Service('/control/vehicle/state', VehicleState, self.state)
        self.pilot_service = rospy.Service('/control/vehicle/pilot', PilotControl, self.pilot)
        self.inhibit_subscriber = rospy.Subscriber('/control/drive/inhibit', Bool, self.setInhibit)
        self.heading_hole_subscriber = rospy.Subscriber('/control/drive/heading_hold', HeadingHold, self.heading_hold)
        self.position_publisher = rospy.Publisher('/sensor/vehicle/position', BasicPositionStamped, queue_size = 5)
        self.heading_publisher = rospy.Publisher('/sensor/vehicle/heading', HeadingStamped, queue_size = 5)
        self.vehicle_status_publisher = rospy.Publisher('/vehicle_status', VehicleStatus, queue_size = 5)
        self.clock_publisher = rospy.Publisher('/clock', Clock, queue_size = 5)
        self.clock_factor_subscriber = rospy.Subscriber('/clock_factor', Float64, self.clock_factor_callback)
        rospy.Timer(rospy.Duration.from_sec(0.05),self.update)
        clock_timer = threading.Timer(self.wallclock_time_step,self.update_clock)
        clock_timer.start()
        rospy.spin()
        
    def state(self,req):
        if req.desired_state in (VehicleStateRequest.VP_STATE_PAUSE, VehicleStateRequest.VP_STATE_ACTIVE):
            self.vehicle_state = req.desired_state
            ret = VehicleStateResponse()
            ret.command_response = VehicleStateResponse.STATE_REQUEST_ACCEPTED
            return ret
        else:
            ret = VehicleStateResponse()
            ret.command_response = VehicleStateResponse.STATE_REQUEST_REJECTED_ILLEGAL_TARGET_STATE
            return ret
    
    def pilot(self,req):
        self.pilot_control = req.control_request
        ret = PilotControlResponse()
        ret.command_response = PilotControlResponse.PILOT_REQUEST_ACCEPTED
        return ret
    
    def setInhibit(self,data):
        self.inhibit = data.data

    def clock_factor_callback(self, data):
        self.time_factor = data.data
    
    def heading_hold(self,data):
        self.last_heading_hold_timestamp = data.header.stamp
        print 'heading hold:', data.thrust.type, data.thrust.value
        # Handle Thrust type==0:Disable, type=1:m/s, type=2:[0-1]
        # Note it is not clear from the documentation if thrust can be <0, 
        # But given that this is a heading-hold message it seems unlikely. 
        if data.thrust.type == 0:
            self.throttle = 0.0
        elif data.thrust.type == 1:
            self.throttle = data.thrust.value / self.dynamics.model["max_speed"]
        elif data.thrust.type == 2:
            self.throttle = data.thrust.value
            
        current_heading = self.dynamics.heading
        heading_delta = data.heading.heading - current_heading
        if heading_delta>math.pi:
            heading_delta -= math.pi*2.0
        if heading_delta<-math.pi:
            heading_delta += math.pi*2.0
        self.rudder = max(-1.0,min(1.0,heading_delta))
        print self.throttle, self.rudder
        
    def update_clock(self):
        self.sim_time += rospy.Duration.from_sec(self.wallclock_time_step*self.time_factor)
        c = Clock(self.sim_time)
        self.clock_publisher.publish(c)
        if not rospy.is_shutdown():
            clock_timer = threading.Timer(self.wallclock_time_step,self.update_clock)
            clock_timer.start()

        
    def update(self, event):
        if self.last_heading_hold_timestamp is None or event.current_real - self.last_heading_hold_timestamp > rospy.Duration.from_sec(0.5):
            hh_expired = True
        else:
            hh_expired = False
        if self.inhibit or self.vehicle_state != VehicleStateRequest.VP_STATE_ACTIVE or not self.pilot_control or hh_expired:
            self.throttle = 0.0
            self.rudder = 0.0
            if self.vehicle_state == VehicleStateRequest.VP_STATE_ACTIVE:
                ss = 'VP_STATE_ACTIVE'
            else:
                ss = 'VP_STATE_PAUSE'
            print 'not in control: inhibit:',self.inhibit,'state:',ss,'pilot control:',self.pilot_control, event.current_real
        
        self.dynamics.update(self.throttle,self.rudder,event.current_real)
        
        p = BasicPositionStamped()
        p.basic_position.position.latitude = math.degrees(self.dynamics.latitude)
        p.basic_position.position.longitude = math.degrees(self.dynamics.longitude)
        p.basic_position.cog = self.dynamics.cog
        p.basic_position.sog = self.dynamics.sog
        p.header.stamp = self.dynamics.last_update
        self.position_publisher.publish(p)
        
        h = HeadingStamped()
        h.header.stamp = self.dynamics.last_update
        h.heading.heading = self.dynamics.heading
        self.heading_publisher.publish(h)
        
        vs = VehicleStatus()
        vs.header.stamp = self.dynamics.last_update
        vs.vehicle_state = self.vehicle_state
        if self.pilot_control:
            if self.inhibit:
                vs.ros_pilot_mode = VehicleStatus.PILOT_INHIBITED
            else:
                if hh_expired:
                    vs.ros_pilot_mode = VehicleStatus.PILOT_INACTIVE
                else:
                    vs.ros_pilot_mode = VehicleStatus.PILOT_HEADING_HOLD
        else:
            vs.ros_pilot_mode = VehicleStatus.PILOT_NOT_IN_COMMAND
        self.vehicle_status_publisher.publish(vs)
        
            
        

if __name__ == '__main__':
    try:
        sim = AsvSim()
        sim.run()
    except rospy.ROSInterruptException:
        pass
