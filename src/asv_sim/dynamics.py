#!/usr/bin/env python

# Roland Arsenault
# Center for Coastal and Ocean Mapping
# University of New Hampshire
# Copyright 2017, All rights reserved.

import math
import random
import geodesic

# Portsmouth, New Hampshire, pier
#start_lat =  43.072091937198394
#start_lon = -70.71126009757178

# Portsmouth, New Hampshire, cod rock
start_lat =  43.073397415457535
start_lon = -70.71054174878898


# Point Hope, Alaska
#start_lat =  68.347
#start_lon = -166.954

class Dynamics:
    def __init__(self,model, environment=None):
        self.model = model
        self.environment = environment
        
        self.reset()
        self.update_coefficients()
        self.last_update = None

        self.jitters = {'thurst':0.1, 'rudder':0.25, 'drag':0.1, 'current_speed':0.1, 'current_direction': 0.25}

    def update_coefficients(self):
        max_prop_speed = (self.model['max_rpm']*self.model['prop_ratio'])/self.model['prop_pitch']
        max_force = self.model['max_power']/self.model['max_speed']
        
        self.prop_coefficient = max_force/(max_prop_speed**2-self.model['max_speed']**2)
        

        self.drag_coefficient = max_force /(self.model['max_speed']**3)
        print 'prop coefficient:',self.prop_coefficient,'drag coefficient:',self.drag_coefficient

        #self.prop_coefficient = (self.model['max_speed']**3)*self.drag_coefficient/self.model['max_rpm']
        


    def reset(self):
        self.rpm = 0
        self.speed = 0.0
        self.longitude = math.radians(start_lon)
        self.latitude = math.radians(start_lat)
        
        self.heading = 1.0
        self.pitch = 0.0
        self.roll = 0.0
        
        self.sog = 0.0
        self.cog = 0.0

    def update(self,throttle, rudder, timestamp):
        #print 'dynamics update:',throttle, rudder, timestamp
        throttle = min(1.0,max(0.0,throttle))
        
        if self.last_update is None:
            delta_t = None
        else:
            delta_t = (timestamp-self.last_update).to_sec()
            if delta_t <= 0:
                delta_t = None
        self.last_update = timestamp

        target_rpm = self.model['idle_rpm']+ throttle * (self.model['max_rpm']-self.model['idle_rpm'])
        if(target_rpm != self.rpm):
            if delta_t is not None:
                rcr = (target_rpm - self.rpm)/delta_t
                if abs(rcr) > self.model['max_rpm_change_rate']:
                    if rcr < 0:
                        rcr = -self.model['max_rpm_change_rate']
                    else:
                        rcr = self.model['max_rpm_change_rate']
                self.rpm += rcr*delta_t

        prop_rpm = self.model['prop_ratio']*self.rpm
        prop_speed = prop_rpm/self.model['prop_pitch']

        if prop_speed > 0:
            rudder_speed = max(math.sqrt(prop_speed),self.speed)
        else:
            rudder_speed = self.speed
            
        thrust = self.prop_coefficient*(prop_speed**2-self.speed**2)
        thrust = random.gauss(thrust,thrust*self.jitters['thrust'])

        rudder_angle = rudder*self.model['max_rudder_angle']
        rudder_angle += random.gauss(0.0,self.jitters['rudder'])
        rudder_rads = math.radians(rudder_angle)

        thrust_fwd = thrust*math.cos(rudder_rads)

        rudder_speed_yaw = rudder_speed*math.sin(rudder_rads)
        yaw_rate = self.model['rudder_coefficient']*rudder_speed_yaw/self.model['rudder_distance']
        if delta_t is not None:
            self.heading += yaw_rate*delta_t
            self.heading = math.fmod(self.heading,math.radians(360))
            if self.heading < 0:
                self.heading += math.radians(360)

        drag = self.speed**3*self.drag_coefficient
        drag = random.gauss(drag,drag*self.jitters['drag'])

        a = (thrust_fwd-drag)/self.model['mass']

        if delta_t is not None:
            self.speed += a*delta_t

        if self.speed > 0:
            (prop_rpm/self.model['prop_pitch'])/self.speed


        if delta_t is not None:
            last_lat = self.latitude
            last_long = self.longitude
            delta = self.speed*delta_t
            self.longitude,self.latitude = geodesic.direct(self.longitude,self.latitude,self.heading,delta)
            if self.environment is not None:
                c = self.environment.getCurrent(self.latitude, self.longitude)
                if c is not None:
                    current_speed = random.gauss(c['speed'],c['speed']*self.jitters['current_speed'])
                    current_direction = math.radians(c['direction']) + random.gauss(0.0,self.jitters['current_direction'])
                    self.longitude,self.latitude = geodesic.direct(self.longitude,self.latitude,current_direction,current_speed*delta_t)
            self.cog, self.sog = geodesic.inverse(last_long, last_lat, self.longitude, self.latitude)
            self.sog /= delta_t
            


        #self.roll = math.radians(math.sin(time.time()) * 2.5)
        #self.pitch = math.radians(math.sin(time.time()/2.1) * 5.0)
        
