# -*- coding: utf-8 -*-
"""
Created on Tue Mar  8 20:58:21 2022

@author: jnguy
"""

import pickle
import re

txt = "PX4_4"
val = (re.findall('\d+', txt ))
#val = [int(s) for s in txt.split() if s.isdigit()]
print(val[1])

print('Waypoint_'+str(val[1]))