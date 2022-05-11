#%%
import numpy as np
import math as math

from numpy import deg2rad, mean

from modules.libnav.interface.interface import NavIface, Tarr2f, Tarr3f
from src.plots import plot_err_formula, plots
from src.sens_data_gen import data_gen
from src.analysis import c_enu_body, rad2meters, rad2min
from numpy.random import normal as rndnorm
import pandas as pd
import matplotlib.pyplot as plt
#%%
"""
    Config section
"""
# e.g Moscow 55.7522200 37.6155600
lat = math.radians(56) # phi
lon = math.radians(0) # lambda
ini_pos = [lat, lon]
# file with real sensors data
sample_time = 90*60 # seconds
data_frequency = 10 # Hz
save_plots = False # plots would be saved to images folder
plots_size = (297,210) # plots height,width in mm

## alignment
heading = math.radians(270)
roll = math.radians(-2)
pitch = math.radians(2)
ini_pry = [pitch, roll, heading]
# time for alignment in seconds
#alignment_time = 60
## alignment

# sensor errors
acc_offset_x = 0.001 * 9.8  # [m/s/s] e.g 1 [mg]
acc_offset_y = 0.001 * 9.8 # [m/s/s] e.g 1 [mg]
gyr_drift_x = math.radians(2)/3600 # 2 [deg/hour]
gyr_drift_y = math.radians(2)/3600 # 2 [deg/hour]
# normal distribution param
sigma_a = 0.5 * 1e-3 * 9.8 # mg
sigma_g = math.radians(0.05) # 0.05 [deg/sec] 
gnss_std = math.radians(3/111111)
gnss_offset = 0#math.radians(0.1/111111)
Tg = 0.2
Ta = 0.3
gnss_TIME = 90
gnss_OFF = 10*60
gnss_ON = gnss_OFF+ 4*60

size = (140/25.4, 170/25.4) # plot size 140, 170 mm
"""
    Config section end
"""

na = NavIface(lat,lon,data_frequency)
G=na.G()
U=na.U()
# задание начальных условий

# расчет матрицы перехода
C = c_enu_body(heading, roll, pitch)

# перепроецируем G из body->enu
a_enu = np.array([
    [0],
    [0],
    [G]
]
)
a_body = C@a_enu
a_x = a_body[0][0];
a_y = a_body[1][0];
a_z = a_body[2][0];

# перепроецируем W земли из body->enu
w_enu = np.array([
    [0],
    [U*math.cos(lat)],
    [U*math.sin(lat)]
]
)
w_body = C@w_enu
w_x = w_body[0][0];
w_y = w_body[1][0];
w_z = w_body[2][0];


#%%
def alg_loop(use_form_filter=True, corr=True, gnss_t=1):
    gnss = [
        rndnorm(lat+gnss_offset, gnss_std, size=sample_time*data_frequency), 
        rndnorm(lon+gnss_offset, gnss_std, size=sample_time*data_frequency),
    ]
    # 0,1,2 A_x,y,z
    # 3,4,5 G_x,y,z
    D = data_gen(use_form_filter,
        [acc_offset_x, acc_offset_y],
        [gyr_drift_x, gyr_drift_y],
        [a_x, a_y, a_z],
        [w_x, w_y, w_z],
        sample_time, data_frequency,
        Ta, Tg,
        sigma_a, sigma_g,
        False
        )
    na.nav().alignment_acc(mean(D[0]), mean(D[1]), mean(D[2]), heading)
    #na.nav().alignment_rph(roll, pitch, heading)
    pry = (
        [0]*(data_frequency*sample_time),
        [0]*(data_frequency*sample_time),
        [0]*(data_frequency*sample_time)
        )
    vel = (
        [0]*(data_frequency*sample_time),
        [0]*(data_frequency*sample_time),
    )
    pos = (
        [0]*(data_frequency*sample_time),
        [0]*(data_frequency*sample_time),
    )
    na.nav().gnss_T(gnss_t)
    na.nav().corr_mode(corr)
    for i in range(0, data_frequency*sample_time):
        if isinstance(D[0], list):
            acc = [(D[0])[i], (D[1])[i], (D[2])[i]]
            gyr = [(D[3])[i], (D[4])[i], (D[5])[i]] 
            g_p = [gnss[0][i], gnss[1][i]]
            na.nav().iter_gnss(
                acc, gyr, g_p
                )
        else:
            acc = [(D[0]), (D[1]), (D[2])]
            gyr = [(D[3]), (D[4]), (D[5])] 
            g_p = [lat+gnss_offset,lon+gnss_offset]
            na.nav().iter_gnss(
                acc, gyr, g_p
                )

        if (i == gnss_OFF*data_frequency and corr):
            na.nav().corr_mode(False)
        if (i == gnss_ON*data_frequency and corr):
            na.nav().corr_mode(True)

        v = Tarr3f()
        na.nav().pry(v)
        for j in range(0,3):
            pry[j][i] = rad2min(v[j] - ini_pry[j])
        na.nav().vel(v)
        for j in range(0,2):
            vel[j][i] = v[j]
        v = Tarr2f()
        na.nav().pos(v)
        for j in range(0,2):
            pos[j][i] = rad2meters(v[j] - ini_pos[j])
    return (pry, vel, pos)


#%%
ALG_DATA_CORR = alg_loop(gnss_t=gnss_TIME, use_form_filter=False, corr=True)
ALG_DATA_NOCORR = alg_loop(gnss_t=gnss_TIME, use_form_filter=False, corr=False)

acc_offset = np.array([
    [acc_offset_x],
    [acc_offset_y],
    [0]
])
acc_err_body = acc_offset
acc_err_enu = C.transpose() @ acc_err_body 
gyr_drift = np.array([
    [gyr_drift_x],
    [gyr_drift_y],
    [0]
])
gyr_drift_body = gyr_drift
gyr_drift_enu = C.transpose() @ gyr_drift_body
EQUAT = plot_err_formula(
    acc_err_enu[0][0],
    acc_err_enu[1][0],
    gyr_drift_enu[0][0],
    gyr_drift_enu[1][0],
    G,
    6378245.0,
    sample_time,
    sample_time*data_frequency,
    heading,
    pitch
) 

#%%
df = pd.DataFrame({
    "Time": np.linspace(0, sample_time/60, sample_time*data_frequency),
    "Fx_corr": ALG_DATA_CORR[0][0],
    "Fy_corr": ALG_DATA_CORR[0][1],
    "Fz_corr": ALG_DATA_CORR[0][2],
    "Ve_corr": ALG_DATA_CORR[1][0],
    "Vn_corr": ALG_DATA_CORR[1][1],
    "phi_corr": ALG_DATA_CORR[2][0],
    "lamda_corr": ALG_DATA_CORR[2][1],
    "Fx_nocorr": ALG_DATA_NOCORR[0][0],
    "Fy_nocorr": ALG_DATA_NOCORR[0][1],
    "Ve_nocorr": ALG_DATA_NOCORR[1][0],
    "Vn_nocorr": ALG_DATA_NOCORR[1][1],
    "phi_nocorr": ALG_DATA_NOCORR[2][0],
    "lamda_nocorr": ALG_DATA_NOCORR[2][1]
})

#%%
# uncomment for interactive plots
%matplotlib
df.plot(
    x="Time", y=["Fx_corr", "Fx_nocorr"],
    grid=True,
    figsize=size,
    #subplots=True,
    #layout=(2,1),
    #xlim=(10,70),
    #ylim=(-50, +50)
)

print("Stab Fx", rad2min(-acc_offset_y/9.8 - na.nav().get_k(1)/(na.nav().get_k(0)+ na.nav().get_k(2))*gyr_drift_x))
#%%
GNSS_T = [i for i in range(45,225, 45)]
for T in GNSS_T:
    inum =5 
    mean_sko_Fx = 0
    mean_sko_Fy = 0
    for j in range(inum):
        angles = alg_loop(gnss_t=T)[0]
        mean_sko_Fx += np.std(angles[0][gnss_ON*data_frequency+4*gnss_TIME*data_frequency:])
        mean_sko_Fy += np.std(angles[1][gnss_ON*data_frequency+4*gnss_TIME*data_frequency:])
    print(
        "T = ", T, '\n'
        "SKO Fx", mean_sko_Fx/inum, '\n'
        "SKO Fy", mean_sko_Fy/inum, '\n'
        "SKO Fxy", math.sqrt(math.pow(mean_sko_Fx/inum, 2)+math.pow(mean_sko_Fy/inum, 2)), '\n'
        )
#%%
"""
print("Maximum difference between equational and alg pitch,roll")
for i in range(0,2):
    print(
        np.max(
        abs(
            EQUAT[0][i] - np.array(pry[i])
        )
        ) /60
    )

print("Maximum difference between equational vel and alg vel v_e, v_n")
for i in range(0,2):
    print(
        np.max(
        abs(
            EQUAT[1][i] - np.array(vel[i])
        )
        )
    )

print("Maximum difference between equational pos and alg pos lat, lon")
for i in range(0,2):
    print(
        np.max(
        abs(
            EQUAT[2][i] - np.array(pos[i])
        )
        ) /60
    )
print("misc")

"""
#%% вычисление ошибок выставки
ae = NavIface(lat,lon, data_frequency)

D = data_gen(False,
    [acc_offset_x, acc_offset_y],
    [gyr_drift_x, gyr_drift_y],
    [a_x, a_y, a_z],
    [w_x, w_y, w_z],
    sample_time, data_frequency,
    Ta, Tg,
    sigma_a, sigma_g,
    False
    )
na.nav().alignment_acc(mean(D[0]), mean(D[1]), mean(D[2]), heading)
res = na.nav().align_prh()
print(math.degrees(res[0]), math.degrees(res[1]), math.degrees(res[2]))
c_pitch = res[0]; c_roll = res[1]

# считаем ошибку, относительно значения из условия
c_roll_err = c_roll-roll
c_pitch_err = c_pitch-pitch

print(
"Ошбики при моделировании выставки в угловых минутах:\n",
"по тангажу ", math.degrees(c_pitch_err)*60, "\n",
"по крену ", math.degrees(c_roll_err)*60, "\n",
)

abx = a_body[0] + acc_offset_x
aby = a_body[1] + acc_offset_y
abz = a_body[2]
dabz = 0
f_pitch_err = acc_offset_y / math.sqrt(math.pow(G,2) - math.pow(aby,2))
f_roll_err = ((abx * dabz) - (acc_offset_x * abz)) / (pow(abx,2) + pow(abz,2))

print(
"Ошибка выставки, полученные по формулам в угловых минутах\n",
"по тангажу ", math.degrees(f_pitch_err)*60, "\n",
"по крену", math.degrees(f_roll_err)*60, "\n",
)
print(
"Разница в определении ошибок в угловых минутах\n",
"по тангажу ", math.degrees(c_pitch_err - f_pitch_err)*60, "\n",
"по крену", math.degrees(c_roll_err - f_roll_err)*60
)

# %%
