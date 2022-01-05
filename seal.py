#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jan  4 16:10:15 2022

@author: davidmann
"""

from os import walk
import numpy as np
import pandas as pd

f = []
mypath = '/Users/davidmann/w/seal/Seal 142/'
filenames = next(walk(mypath), (None, None, []))[2]  # [] if no file
#print(filenames)

dfDive = pd.DataFrame()
filenames.sort()
for filename in filenames:
    if(filename[0]!='.'):
        # filename = 'seal142_210226T144803.csv'
        #filename = 'seal50_210301T044600.csv'
        print(filename)
        depthThreshold = 300
        ascentThreshold = 100
        timeInterval = 10
        
        df = pd.read_csv(mypath + filename).dropna()
        df = df.set_index('date')
        dfDive = dfDive.append(df)
#dfDive.plot(ylabel='Depth m', title=filename + ' sec:' + str(timeInterval) + ' DT:' + str(depthThreshold) + ' AT:' + str(ascentThreshold))
dfResample = dfDive.iloc[::timeInterval]
dfResample['depth'].plot(style='.', ylabel='Depth m', title=mypath + ' sec:' + str(timeInterval) + ' DT:' + str(depthThreshold) + ' AT:' + str(ascentThreshold))
dfDiff = -dfResample.diff(periods=18)
#dfDiff['depth'].plot(style='.')

dfPlayback = dfResample[(dfResample > depthThreshold) & (dfDiff>ascentThreshold)]
dfPlayback['depth'].plot(style='.')

        