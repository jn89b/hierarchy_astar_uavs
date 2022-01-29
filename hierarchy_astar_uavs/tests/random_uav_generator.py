# -*- coding: utf-8 -*-
"""
Created on Fri Jan 28 19:23:38 2022

@author: jnguy
"""

## generate n number of coordinates[x,y,z] that are unique and do not belong to some number

import random

radius = 2
rangeX = (0, 10)
rangeY = (0, 10)
rangeZ = (0, 10)
qty = 5  # or however many points you want

obstacles = set([1,2,1])

# Generate a set of all points within 200 of the origin, to be used as offsets later
# There's probably a more efficient way to do this.
deltas = set()
for x in range(-radius, radius+1):
    for y in range(-radius, radius+1):
        for z in range(-radius, radius+1):
            if x*x + y*y+ z*z <= radius*radius:
                if (x,y,z) in obstacles:
                    print("in obstacles")
                    val = (x,y,z)
                    continue
                else:
                    deltas.add((x,y,z))

randPoints = []
excluded = set()
i = 0
while i<qty:
    x = random.randrange(*rangeX)
    y = random.randrange(*rangeY)
    z = random.randint(*rangeZ)
    if (x,y,z) in excluded or (x,y,z) in obstacles: 
        continue
    randPoints.append((x,y,z))
    i += 1
    excluded.update((x+dx, y+dy, z+dy) for (dx,dy,dz) in deltas)

print(randPoints)
