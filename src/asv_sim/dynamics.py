#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Roland Arsenault
# Center for Coastal and Ocean Mapping
# University of New Hampshire
# Copyright 2017, All rights reserved.

from __future__ import absolute_import
from builtins import object
import math
import random
from . import geodesic

# Portsmouth, New Hampshire, pier
#start_lat =  43.072091937198394
#start_lon = -70.71126009757178

# Portsmouth, New Hampshire, cod rock
start_lat =  43.073397415457535
start_lon = -70.71054174878898
start_heading = 60.0

# Point Hope, Alaska
#start_lat =  68.347
#start_lon = -166.954

class Dynamics(object):
    """Simulate the dynamics of a boat.
    
    Using throttle and rudder inputs, calculate simplified forces acting on a boat changing its
    position, speed, and heading.
    """
    
    def __init__(self,model, environment=None, start=None):
        """Initialize the simulation with a model and optional environment.
        
        The model parameter is a dictionary containing values specific to the boat being simulated.
        - max_rpm: The maximum engine speed achieved using full throttle.
        - max_power: Power in watts produced at max_rpm.
        - idle_rpm: Lowest rpm reached when throttle is at 0.
        - clutch_engagement_rpm: RPM at which the clutch engages. Should set to 0 if alwas engaged.
        - gearbox_ratio: Ratio of prop rpm relative to engine rpm.
        - propulsion_type: "jet" or "prop"
        - prop_pitch: TODO: figure out how this is used, web says inches traveled assuming no slip for each revolution.
        - bucket_efficency: For jet drive with reverse bucket, how much reverse thrust relative to forward.
        - max_bucket_change_rate: How fast the bucket can move in full deflections per second.
        - max_rpm_change_rate: Represents engine inertia by limiting how much rpms can change per second.
        - max_speed: Maximum speed of the boat in m/s.
        - mass: Mass of the boat in kilograms.
        - max_rudder_angle: Maximum deflection of the rudder in degrees.
        - max_turn_rate: Maximum rate the boat can turn in degrees per second.
        - rudder_distance: Distance between the rudder and the boat's center of mass in meters.
        - rudder_coefficient: Expresses rudder efficency. TODO: Elaborate on what this means.
        
        The environment parameter is optional. TODO: add details
        
        """
        global start_lat
        global start_lon
        global start_heading

        if start is not None:
            if 'lat' in start:
                start_lat = start['lat']
            if 'lon' in start:
                start_lon = start['lon']
            if 'lat' in start:
                start_heading = start['heading']

        self.model = model
        self.environment = environment
        
        self.reset()
        self.update_coefficients()
        self.last_update = None

        # jitters are used to add gaussian noise to components of the model to simulate small
        # disturbances that would be difficult to model.
        self.jitters = {'thrust':0.1, 'rudder':0.25, 'drag':0.1, 'current_speed':0.1, 'current_direction': 0.25}


    def reset(self):
        """Resets the simulation to starting location and idle state."""
        
        self.rpm = 0
        self.rudder_angle = 0.0
        self.bucket_position = 0.0
        self.speed = 0.0
        self.yaw_rate = 0.0
        self.longitude = math.radians(start_lon)
        self.latitude = math.radians(start_lat)

        self.heading = math.radians(start_heading)
        self.pitch = 0.0
        self.roll = 0.0

        self.sog = 0.0
        self.cog = 0.0

    def set(self, lat, lon, heading):
        """Sets the boat to idle at the provded position and heading."""
        
        self.rpm = 0
        self.rudder_angle = 0.0
        self.bucket_position = 0.0
        self.speed = 0.0
        self.yaw_rate = 0.0
        self.longitude = math.radians(lon)
        self.latitude = math.radians(lat)

        self.heading = heading
        self.pitch = 0.0
        self.roll = 0.0

        self.sog = 0.0
        self.cog = 0.0

    def update_coefficients(self):
        """Calculates coefficents that are used to balance out the carious limits.
        
        The engine max power and the boat's top speed are used to calculate a maximum force.
        This maximum force is used to calculate a drag coefficient that results in forces
        being in equilibrium at top speed and rpm.
        """

        self.max_force = self.model['max_power']/self.model['max_speed']

        self.drag_coefficient = self.max_force /(self.model['max_speed']**3)
        self.max_prop_rpm = self.model['max_rpm']*self.model['gearbox_ratio']

        if self.model['propulsion_type'] == 'prop':
            max_prop_speed = self.max_prop_rpm/self.model['prop_pitch']
        
            # see comment in update related to thrust calculation for more info.
            self.prop_coefficient = self.max_force/(max_prop_speed**2-self.model['max_speed']**2)

        if self.model['propulsion_type'] == 'jet':
            # power = a*rpm^3
            # a = power/rpm^3
            #self.jet_coefficient = self.model['max_power']/pow(self.max_prop_rpm,3)
            
            #https://www.engineeringtoolbox.com/jet-discharge-propulsion-force-d_1868.html
            # The propulsive force or thrust induced by the jet can be expressed as
            # F = ρ q (v2 - v1)
            # where v1 = jet velocity (m/s), v2 = velocity out of the jet (m/s), rho is density and q is volume
            
            max_turn_force = self.max_force*math.sin(math.radians(self.model['max_rudder_angle']))

            # analagous to straight line drag coefficient, but to resist turning          
            # linear works better than cubic, this the **1
            self.go_straight_coefficient = max_turn_force / (math.radians(self.model['max_turn_rate'])**1)
            
            
            
    def update(self,throttle, rudder, timestamp):
        """Updates the simulation one step.
        
        The simulation does not impose an update rate or expect that
        updates are at regular intervals. It's the system using this
        model that is responsible for continuously calling this
        method at an adequate rate.
        
        :param throttle: Normalized throttle value in the 0 to 1 range.
        :param rudder: Normalized rudder value from -1 (left)  to 1 (right).
        :param timestamp: Time in seconds for the current iteration. It only 
            matters that the time reference is consistant, since it's only
            used to calculate elapsed time since the last update.
        """
        
        # collect usefull debugging info
        diagnostics = {'rudder':rudder}
        
        # clamp throttle input
        throttle = min(1.0,max(-1.0,throttle))
        reverse = throttle < 0
        throttle = abs(throttle)
        diagnostics['throttle'] = throttle
        
        # Figure out the time inteval since the last update
        if self.last_update is None:
            delta_t = None
        else:
            delta_t = (timestamp-self.last_update).to_sec()
            if delta_t <= 0:
                delta_t = None
        self.last_update = timestamp

        # This implements the inertia of the engine as it changes RPMs.
        target_rpm = self.model['idle_rpm']+ throttle * (self.model['max_rpm']-self.model['idle_rpm'])
        diagnostics['target_rpm'] = target_rpm
        if(target_rpm != self.rpm):
            if delta_t is not None:
                rcr = (target_rpm - self.rpm)/delta_t
                if abs(rcr) > self.model['max_rpm_change_rate']:
                    if rcr < 0:
                        rcr = -self.model['max_rpm_change_rate']
                    else:
                        rcr = self.model['max_rpm_change_rate']
                self.rpm += rcr*delta_t
                
        diagnostics['rpm'] = self.rpm        

        # Apply the gearbox and clutch.
        if self.rpm < self.model['clutch_engagement_rpm']:
            prop_rpm = 0
        else:
            prop_rpm = self.model['gearbox_ratio']*self.rpm

        diagnostics['prop_rpm'] = prop_rpm

        
        target_rudder_angle = rudder*self.model['max_rudder_angle']
        diagnostics['target_rudder_angle'] = target_rudder_angle
        
        if(self.rudder_angle != target_rudder_angle and delta_t is not None):
            possible_rudder_change = self.model['max_rudder_change_rate']*delta_t
            if abs(target_rudder_angle-self.rudder_angle) < possible_rudder_change:
                self.rudder_angle = target_rudder_angle
            else:
                if (target_rudder_angle < self.rudder_angle):
                    self.rudder_angle -= possible_rudder_change
                else:
                    self.rudder_angle += possible_rudder_change

        diagnostics['rudder_angle'] = self.rudder_angle
        rudder_rads = math.radians(self.rudder_angle + random.gauss(0.0,self.jitters['rudder']))

        if self.model['propulsion_type'] == 'prop':
            # Calculate the speed of the water moved by the prop.
            # TODO: figure out why this seems to differe from the definition
            # of pitch as being inches of distance traveled per revolution
            prop_speed = prop_rpm/self.model['prop_pitch']

            # Assuming the rudder is in the prop wash, determine the speed of the
            # water flowing by the rudder.
            if prop_speed > 0:
                rudder_speed = max(math.sqrt(prop_speed),self.speed)
            else:
                rudder_speed = self.speed

            # Calculate force created by the difference of the speed of
            # the water pushed by the prop and the speed of the boat
            # through water.
            #
            # From https://wright.nasa.gov/airplane/propth.html
            #
            # F = .5 * r * A * [Ve ^2 - V0 ^2]
            # 
            # Replace .5 * r * A by prop_coefficient to get the following
            thrust = self.prop_coefficient*(prop_speed**2-self.speed**2)
            
        if self.model['propulsion_type'] == 'jet':

            # https://www.engineeringtoolbox.com/affinity-laws-d_408.html
            # flow is directly proportional to rpm
            
            #jet_speed = self.max_jet_speed*prop_rpm/self.max_prop_rpm
            #diagnostics['jet_speed'] = jet_speed
            
            # F = ρ q (v2 - v1)
            #thrust = max(0.0,self.rho_q*(jet_speed-self.speed))
            
            thrust = self.max_force*prop_rpm/self.max_prop_rpm
            
            # todo: model bucket movement
            if reverse:
                thrust = -thrust*self.model['bucket_efficency']
            
            
        thrust = random.gauss(thrust,thrust*self.jitters['thrust'])
        diagnostics['thrust'] = thrust
        
        diagnostics['max_force'] = self.max_force


        # decompose the thrust into vectors that push the boat
        # forward and that rotates the boat
        thrust_fwd = thrust*math.cos(rudder_rads)

        if self.model['propulsion_type'] == 'prop':
            rudder_speed_yaw = rudder_speed*math.sin(rudder_rads)
            # The distance between the center of mass and the rudder affect how quickly the boat can turn.
            self.yaw_rate = self.model['rudder_coefficient']*rudder_speed_yaw/self.model['rudder_distance']

        if self.model['propulsion_type'] == 'jet':
            yaw_thrust = thrust*math.sin(rudder_rads)
            
            yaw_drag = -(self.yaw_rate**1)*self.go_straight_coefficient
            
            diagnostics['yaw_thrust'] = yaw_thrust
            diagnostics['yaw_drag'] = yaw_drag
            
            torque = (yaw_thrust+yaw_drag)*self.model['rudder_distance']
            
            # https://en.wikipedia.org/wiki/Moment_of_inertia
            # torque = I * angular acceleration
            
            # let's oversimplify the asv to a thin disc
            # I = 1/2 * m * r^2
            
            moment_of_inertia = self.model['mass']*pow(self.model['rudder_distance'],2)/2.0
            
            angular_acceleration = torque/moment_of_inertia
            
            if delta_t is not None:
                self.yaw_rate += angular_acceleration*delta_t
            

        diagnostics['yaw_rate'] = self.yaw_rate
            
        if delta_t is not None:
            self.heading += self.yaw_rate*delta_t
            self.heading = math.fmod(self.heading,math.radians(360))
            if self.heading < 0:
                self.heading += math.radians(360)

        # Calculate drag increasing with the cube of the speed.
        drag = self.speed**3*self.drag_coefficient
        drag = random.gauss(drag,drag*self.jitters['drag'])

        # calculate acceleration from f=ma.
        self.a = (thrust_fwd-drag)/self.model['mass']

        if delta_t is not None:
            self.speed += self.a*delta_t

        # update position based on speed
        if delta_t is not None:
            last_lat = self.latitude
            last_long = self.longitude
            delta = self.speed*delta_t
            self.longitude,self.latitude = geodesic.direct(self.longitude,self.latitude,self.heading,delta)
            # apply currents if applicable
            if self.environment is not None:
                c = self.environment.getCurrent(self.latitude, self.longitude)
                if c is not None:
                    current_speed = random.gauss(c['speed'],c['speed']*self.jitters['current_speed'])
                    current_direction = math.radians(c['direction']) + random.gauss(0.0,self.jitters['current_direction'])
                    self.longitude,self.latitude = geodesic.direct(self.longitude,self.latitude,current_direction,current_speed*delta_t)
            self.cog, self.sog = geodesic.inverse(last_long, last_lat, self.longitude, self.latitude)
            self.sog /= delta_t
            
        return diagnostics
        
