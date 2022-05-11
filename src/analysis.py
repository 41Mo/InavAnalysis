from . import plots
import math
from numpy import array
from math import degrees as deg

def rad2min(d):
    return deg(d)*60

def rad2meters(d):
    return deg(d)*111111

def plot_model(self, DATA, title="", save=False, err=False):
    plots(DATA, self.time, self.points, title=title, save=save, err=err)

#def make_err_model(self):
#    for i in range(0, self.points):
#        roll[i] = d.roll[i] - self.roll
#        pitch[i] = d.pitch[i] - self.pitch
#        yaw[i] = d.yaw[i] - self.yaw
#        v_e[i] = d.v_e[i]
#        v_n[i] = d.v_n[i]
#        lat[i] = d.lat[i] - self.lat
#        lon[i] = d.lon[i] - self.lon
#    return OUT(roll ,pitch, yaw, lat, lon, v_e, v_n)

def c_enu_body(yaw, roll, pitch):
    psi = yaw
    teta= pitch;
    gamma= roll;

    sp = math.sin(psi)
    st = math.sin(teta)
    sg = math.sin(gamma)

    cp = math.cos(psi)
    ct = math.cos(teta)
    cg = math.cos(gamma)

    a11 = cp*cg + sp*st*sg
    a12 = sp*ct
    a13 = cp*sg - sp*st*cg
    a21 = -sp*cg + cp*st*sg
    a22 = cp*ct
    a23 = -sp*sg - cp*st*cg
    a31 = -ct*sg
    a32 = st
    a33 = ct*cg

    # body_enu matrix
    C_body_enu= array([
        [a11, a12, a13],
        [a21, a22, a23],
        [a31, a32, a33]
    ])

    # enu to body matrix
    return C_body_enu.transpose()