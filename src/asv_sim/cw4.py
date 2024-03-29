#!/usr/bin/env python3

# Roland Arsenault and Val Schmidt
# Center for Coastal and Ocean Mapping
# University of New Hampshire
# Copyright 2017, All rights reserved.

''' Model for the C-Worker 4 ASV

Engine: 30 bHP -> *.4 for losses * 745 to watts = 8948.4
3200 rpm: 30 bHP
Maximum output at crankshaft: 21.3 kW@3200 rpm
draft 0.4m
4m long, 1.7 wide, and has a draft of 0.4 m.
Tonnage: 1 T
Top Speed:       5.5 knots (2.75 m/s)
Minimum survey speed:    1 knots
Propulsion:      1 x 30 HP Yanmar Diesel direct drive, 3YM-30E.

Jet drive: Alamarin AJ-160
  Graph for the Alamarin line shows power (kW) to RPM graph that looks like
  the equation is y=a(bx)^3 where x is rpm, y is power (kW), b is 1/5000 and
  a ranges from 80 to 120.
  Using a = 100, and rearanging to y=cx^3, c = 0.0000000008
  

'''
cw4 = {'max_rpm':3200,
       'max_power':21300,
       'idle_rpm':800,
       'clutch_engagement_rpm':1000,
       'gearbox_ratio':1.0,
       'propulsion_type':'jet',
       'bucket_efficency':0.2,
       'max_bucket_change_rate':0.5,
       'max_rpm_change_rate':1000,    # FIX
       'max_speed':2.75,
       'mass':2000,                   
       'max_rudder_angle':33,
       'max_rudder_change_rate':30,
       'max_turn_rate':25,
       'rudder_distance':2,
       }

