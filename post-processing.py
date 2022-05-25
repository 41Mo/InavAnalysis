#%%
import math
from modules.libnav.interface.interface import NavIface
import pandas as pd
import numpy as np
from numpy.random import normal as rndnorm
#%matplotlib widget
import matplotlib.pyplot as plt

df = pd.read_csv('csv_data/101003-0757Saratov.txt', delimiter='  ')

#%%
#df.plot( y=["Acc_X"], grid=True, linewidth=2)
acc = df.loc[:, ["Acc_X", "Acc_Y", "Acc_Z"]].to_numpy()
gyr = df.loc[:, ["Gyr_X", "Gyr_Y", "Gyr_Z"]].to_numpy()

#%%
lat = math.radians(55.7522200)
lon = math.radians(37.6155600)
freq = 100
alignemnt_points = 30 * freq
gnss_std = math.radians(3/111111)/math.sqrt(1/freq)
gnss_T = 360
gnss_OFF = 10*60
gnss_ON = (gnss_OFF+ 5*60)
gnss_OFF *= freq
gnss_ON *= freq
test_mid_off = False

mag_yaw = math.radians(df["Yaw"][alignemnt_points])
if mag_yaw < 0:
    mag_heading = mag_yaw + 2*math.pi
else:
    mag_heading = mag_yaw
points = len(acc)

time_min = len(acc[:,0])/freq/60
align,acc = np.split(acc, [alignemnt_points])
gyr = np.split(gyr, [alignemnt_points])[1]


gnss = rndnorm((lat, lon), gnss_std, size=(len(acc[:,0]), 2)),
gnss = np.squeeze(gnss)
#%%
ni = NavIface(lat, lon, freq)
ni.nav().alignment_acc(
    np.mean(align[:,0]),
    np.mean(align[:,1]),
    np.mean(align[:,2]),
    mag_heading
)

ni.nav().gnss_T(gnss_T)
ni.nav().corr_mode(True)
#%%
pry = []; vel = []; pos = []
i=0
for a, g, sns in zip(acc, gyr, gnss):
    if i == gnss_OFF and test_mid_off:
        ni.nav().corr_mode(False)
    elif i == gnss_ON and test_mid_off:
        ni.nav().corr_mode(True)

    ni.nav().iter_gnss(
        a, g, sns
    )
    pry.append(ni.nav().get_pry())
    vel.append(ni.nav().get_vel())
    pos.append(ni.nav().get_pos())
    i+=1
#%%
def pry2prh(pry):
    if pry[2] < 0:
        pry[2] += 2*math.pi
    return pry

#%%
pry = [pry2prh(e) for e in pry]
pry = np.rad2deg(pry)
vel = np.array(vel)
pos = np.rad2deg(pos)

df2 = pd.DataFrame(
    {
        "Time": np.linspace(alignemnt_points/freq/60, time_min, points-alignemnt_points),
        "Pitch": pry[:,0],
        "Roll": pry[:,1],
        "Hdg": pry[:,2],
        "V_e": vel[:,0],
        "V_n": vel[:,1],
        "Lat": pos[:,0],
        "Lon": pos[:,1]
    }
)

#%%
size = (140/25.4, 170/25.4)
# To get a list of all of the styles available from Mathplotlib use the following command.
# plt.style.available
plt.style.use('fivethirtyeight')
df2.plot(
    x="Time", y=["Pitch", "Roll", "Hdg"],
    grid=True,
    figsize=size,
    subplots=True,
    layout=(3,1),
    linewidth=2
)
# %%
df2.plot(
    x="Time", y=["V_e", "V_n"],
    grid=True,
    figsize=size,
    subplots=True,
    layout=(2,1),
    #xlim=(0,10),
    #ylim=(-2000, +2000)
)

# %%
df2.plot(
    x="Time", y=["Lat", "Lon"],
    grid=True,
    figsize=size,
    subplots=True,
    layout=(2,1),
    #xlim=(0,10),
    #ylim=(-2000, +2000)
)
# %%
