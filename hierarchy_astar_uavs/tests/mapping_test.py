# -*- coding: utf-8 -*-
"""
Created on Fri Jan 21 12:49:08 2022

@author: jnguy
"""

import numpy as np

np.random.seed(123)

# sources = range(100)
# outs = [a for a in range(100)]
# np.random.shuffle(outs)
# mapping = {sources[a]:outs[a] for a in(range(len(sources)))}


# k = np.array(list(mapping.keys()))
# v = np.array(list(mapping.values()))

# mapping_ar = np.zeros(k.max()+1,dtype=v.dtype) #k,v from approach #1
# mapping_ar[k] = v
# #out = mapping_ar[input_array]

for i in range(0,10,5):
    print(i)
