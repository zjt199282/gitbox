#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 20 21:31:33 2018

@author: zjt
"""

from time import time

def timer(func):
    def f(*args,**kwargs):
        before = time()
        rv = func(*args, **kwargs)
        after = time()
        print('elapsed', after - before)
        return rv
    return f