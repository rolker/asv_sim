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
    """Simulate the dynamics of a boat.
    
    Using throttle and rudder inputs, calculate simplified forces acting on a boat changing its
    position, speed, and heading.
    """
    
    def __init__(self,model, environment=None):
        """Initialize the simulation with a model and optional environment.
        
        The model parameter is a dictionary containing values specific to the boat being simulated.
        - max_rpm: The maximum engine speed achieved using full throttle.
        - max_power: Power in watts produced at max_rpm.
        - idle_rpm: Lowest rpm reached when throttle is at 0.
        - prop_ratio: Ratio of prop rpm relative to engine rpm.
        - prop_pitch: TODO: figure out how this is used, web says inches traveled assuming no slip for each revolution.
        - max_rpm_change_rate: Represents engine inertia by limiting how much rpms can change per second.
        - max_speed: Maximum speed of the boat in m/s.
        - mass: Mass of the boat in kilograms.
        - max_rudder_angle: Maximum deflection of the rudder in degrees.
        - rudder_distance: Distance between the rudder and the boat's center of mass in meters.
        - rudder_coefficient: Expresses rudder efficency. TODO: Elaborate on what this means.
        
        The environment parameter is optional. TODO: add details
        
        """
        
        self.model = model
        self.environment = environment
        
        self.reset()
        self.update_coefficients()
        self.last_update = None

        # jitters are used to add gaussian noise to components of the model to simulate small
        # disturbances that would be difficult to model.
        self.jitters = {'thrust':0.1, 'rudder':0.25, 'drag':0.1, 'current_speed':0.1, 'current_direction': 0.25}

    def update_coefficients(self):
        """Calculates coefficents that are used to balance out the carious limits.
        
        The engine max power and the boat's top speed are used to calculate a maximum force.
        This maximum force is used to calculate a drag coefficient that results in forces
        being in equilibrium at top speed and rpm.
        """
        
        max_prop_speed = (self.model['max_rpm']*self.model['prop_ratio'])/self.model['prop_pitch']
        max_force = self.model['max_power']/self.model['max_speed']
        
        # see comment in update related to thrust calculation for more info.
        self.prop_coefficient = max_force/(max_prop_speed**2-self.model['max_speed']**2)
        

        self.drag_coefficient = max_force /(self.model['max_speed']**3)
        print 'prop coefficient:',self.prop_coefficient,'drag coefficient:',self.drag_coefficient

        #self.prop_coefficient = (self.model['max_speed']**3)*self.drag_coefficient/self.model['max_rpm']
        


    def reset(self):
        """Resets the simulation to starting location and idle state."""
        
        self.rpm = 0
        self.speed = 0.0
        self.longitude = math.radians(start_lon)
        self.latitude = math.radians(start_lat)

        self.heading = 1.0
        self.pitch = 0.0
        self.roll = 0.0

        self.sog = 0.0
        self.cog = 0.0

    def set(self, lat, lon, heading):
        """Sets the boat to idle at the provded position and heading."""
        
        self.rpm = 0
        self.speed = 0.0
        self.longitude = math.radians(lon)
        self.latitude = math.radians(lat)

        self.heading = heading
        self.pitch = 0.0
        self.roll = 0.0

        self.sog = 0.0
        self.cog = 0.0

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
        
        #print 'dynamics update:',throttle, rudder, timestamp
        throttle = min(1.0,max(0.0,throttle))
        
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
        if(target_rpm != self.rpm):
            if delta_t is not None:
                rcr = (target_rpm - self.rpm)/delta_t
                if abs(rcr) > self.model['max_rpm_change_rate']:
                    if rcr < 0:
                        rcr = -self.model['max_rpm_change_rate']
                    else:
                        rcr = self.model['max_rpm_change_rate']
                self.rpm += rcr*delta_t

        # Apply the gearbox.
        prop_rpm = self.model['prop_ratio']*self.rpm

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
        thrust = random.gauss(thrust,thrust*self.jitters['thrust'])

        rudder_angle = rudder*self.model['max_rudder_angle']
        rudder_angle += random.gauss(0.0,self.jitters['rudder'])
        rudder_rads = math.radians(rudder_angle)

        # decompose the thrust into vectors that push the boat
        # forward and that rotates the boat
        thrust_fwd = thrust*math.cos(rudder_rads)

        rudder_speed_yaw = rudder_speed*math.sin(rudder_rads)
        # The distance between the center of mass and the rudder affect how quickly the boat can turn.
        yaw_rate = self.model['rudder_coefficient']*rudder_speed_yaw/self.model['rudder_distance']
        if delta_t is not None:
            self.heading += yaw_rate*delta_t
            self.heading = math.fmod(self.heading,math.radians(360))
            if self.heading < 0:
                self.heading += math.radians(360)

        # Calculate drag increasing with the cube of the speed.
        drag = self.speed**3*self.drag_coefficient
        drag = random.gauss(drag,drag*self.jitters['drag'])

        # calculate acceleration from f=ma.
        a = (thrust_fwd-drag)/self.model['mass']

        if delta_t is not None:
            self.speed += a*delta_t

        # TODO: figure out if the following is a bug or old code that should be cleaned away
        if self.speed > 0:
            (prop_rpm/self.model['prop_pitch'])/self.speed

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
            


        #self.roll = math.radians(math.sin(time.time()) * 2.5)
        #self.pitch = math.radians(math.sin(time.time()/2.1) * 5.0)
        
