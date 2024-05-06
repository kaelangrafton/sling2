# -*- coding: utf-8 -*-
"""
Created on Tue Jan 24 21:03:56 2023

@author: kaela
"""

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

df = pd.read_csv('c172_shotperiod.txt', sep='|', header=0)

df.rename(columns = lambda x: x.replace(',',''), inplace = True)
df.rename(columns = lambda x: x.replace(' ',''), inplace = True)
df.rename(columns = lambda x: x.replace('__','_'), inplace = True)


plt.plot(df['missn_time'], df['pitch_deg'], label='pitch_deg')
plt.plot(df['missn_time'], df['_Vind_keas'], label='_Vind_keas')
plt.xlabel('missn_time')
plt.ylabel('Value')
plt.legend()
plt.show()