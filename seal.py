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
mypath = '/Users/davidmann/w/seal/Seal 159/'
filenames = next(walk(mypath), (None, None, []))[2]  # [] if no file
#print(filenames)



dfDive = pd.DataFrame()
filenames.sort()
for filename in filenames:
    if(filename[0]!='.'):
        # filename = 'seal142_210226T144803.csv'
        #filename = 'seal50_210301T044600.csv'
        print(filename)

        
        df = pd.read_csv(mypath + filename).dropna()
        df = df.set_index('date')
        dfDive = dfDive.append(df)
#dfDive.plot(ylabel='Depth m', title=filename + ' sec:' + str(timeInterval) + ' DT:' + str(depthThreshold) + ' AT:' + str(ascentThreshold))


depthThreshold = 275
ascentThreshold = 100
timeInterval = 10
minPlaybackIntervalMin = 540 

dfResample = dfDive.iloc[::timeInterval]['depth']
dfResample.index = pd.to_datetime(dfResample.index, format='%y-%m-%dT%H:%M:%S', yearfirst=True)
dfResample.plot(style='.', ylabel='Depth m')
dfDiff = -dfResample.diff(periods=18)
dfDiff.plot(style='.')

dfPlayback = dfResample[(dfResample > depthThreshold) & (dfDiff>ascentThreshold)]
dfPlayback.plot(style='.')


# clear playbacks too close together
dfP = dfPlayback.dropna()
# get indices of good playbacks
lastGoodPlayback = dfP.index[0]
goodPlaybacks = pd.Series(dfP[0], index = [lastGoodPlayback])
for i in range(2, len(dfPlayback)):
    dt = dfP.index[i] - lastGoodPlayback
    if (dt.total_seconds()/60)>minPlaybackIntervalMin:
        lastGoodPlayback = dfP.index[i]
        newRow = pd.Series([dfP[i]], index = [lastGoodPlayback])
        goodPlaybacks = goodPlaybacks.append(newRow)

len(goodPlaybacks)
goodPlaybacks.plot(style='o', color='red',  title=mypath + ' sec:' + str(timeInterval) + ' DT:' + str(depthThreshold) + ' AT:' + str(ascentThreshold) + ' Playbacks=' + str(len(goodPlaybacks)))

        