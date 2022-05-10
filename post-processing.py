#%%
import math
from modules.libnav.interface.interface import NavIface
import pandas as pd
import numpy as np
%matplotlib
import matplotlib.pyplot as plt

df = pd.read_csv('csv_data/Sensors_and_orientation.csv', delimiter=';')
#%%
acc = df.loc[:, ["Acc_X", "Acc_Y", "Acc_Z"]].to_numpy()
gyr = df.loc[:, ["Gyr_X", "Gyr_Y", "Gyr_Z"]].to_numpy()

#%%
lat = math.radians(55.7522200)
lon = math.radians(37.6155600)
freq = 100
alignemnt_time = 30 * freq
mag_yaw = math.radians(df["Yaw"][alignemnt_time])
if mag_yaw < 0:
    mag_heading = mag_yaw + 2*math.pi
else:
    mag_heading = mag_yaw
points = len(acc)

time_min = len(acc[:,0])/freq/60
align,data = np.split(acc, [alignemnt_time])

#%%
ni = NavIface(lat, lon, freq)
ni.nav().alignment_acc(
    np.mean(align[:,0]),
    np.mean(align[:,1]),
    np.mean(align[:,2]),
    mag_heading
)
#%%
pry = []; vel = []; pos = []
for a, g in zip(acc, gyr):
    ni.nav().iter(
        a[0], a[1], a[2],
        g[0], g[1], g[2]
    )
    pry.append(ni.nav().get_pry())
    vel.append(ni.nav().get_vel())
    pos.append(ni.nav().get_pos())
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
        "Time": np.linspace(0, time_min, points),
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
