import numpy as np
from numpy import cos as cos, ndarray
from numpy import sin as sin
from numpy import tan as tan
import math as math
from src.pycore.structs import *
import logging
logger = logging.getLogger(__name__)
class nav_alg:

    # Earth parameters
    R = 6378245.0; # earth radius [m]
    U = math.radians(15)/3600 # earth speed [rad/sec]
    G = 9.81 # [m/s/s]

    def __init__(self):

        self._H = 0.0 # object height above ground
        self._w_body = vec_body() # angle speed body
        self._a_body = vec_body() # acceleration body
        self._v_enu  = vec_enu() # linear speed enup
        self._coord   = vec_coordinates() # lat, lon (phi, lambda)
        self._tm_body_enu = np.eye(3, dtype=np.double) # transformation matrix body - enup
        self._w_enu = vec_enu(
            0,
            self.U*math.cos(self._coord.phi),
            self.U*math.sin(self._coord.phi)
        ) # angle speed enup
        self._a_enu = vec_enu(0,0,self.G) # acceleration enup
        self._rph_angles  = vec_orientation() # euler angles


    def _puasson_equation(self):
        wbx = self._w_body.x
        wby = self._w_body.y
        wbz = self._w_body.z

        wox = self._w_enu.E
        woy = self._w_enu.N
        woz = self._w_enu.Up
        C = self._tm_body_enu

        w_body = np.array([
            [   0, -wbz,  wby],
            [ wbz,    0, -wbx],
            [-wby,  wbx,    0]
        ], dtype=np.double)

        w_enu = np.array([
            [   0, -woz,  woy],
            [ woz,    0, -wox],
            [-woy,  wox,    0]
        ], dtype=np.double)

        self._tm_body_enu = C + (C @ w_body  - w_enu @ C) * self.dt
        if logger.isEnabledFor(logging.DEBUG):
            logger.debug(f'C_body_enu\n{self._tm_body_enu}')

    def _euler_angles(self):

        C = self._tm_body_enu

        C_0 = np.sqrt(pow(C[0,2],2) + pow(C[2,2],2))

        self._rph_angles.teta = np.arctan(C[1,2]/C_0)
        self._rph_angles.gamma = -np.arctan(C[0,2]/C[2,2])
        self._rph_angles.psi = np.arctan(C[1,0]/C[1,1])

        if logger.isEnabledFor(logging.DEBUG):
            logger.debug(f'Roll,Pitch,Heading\n {self._rph_angles}')

    def _acc_body_enu(self):

        c11 = self._tm_body_enu[0,0]; c12 = self._tm_body_enu[0,1]; c13 = self._tm_body_enu[0,2]
        c21 = self._tm_body_enu[1,0]; c22 = self._tm_body_enu[1,1]; c23 = self._tm_body_enu[1,2]
        c31 = self._tm_body_enu[2,0]; c32 = self._tm_body_enu[2,1]; c33 = self._tm_body_enu[2,2]

        self._a_enu.E = c11 * self._a_body.x + c12 * self._a_body.y + c13 * self._a_body.z;
        self._a_enu.N = c21 * self._a_body.x + c22 * self._a_body.y + c23 * self._a_body.z;
        self._a_enu.U = c31 * self._a_body.x + c32 * self._a_body.y + c33 * self._a_body.z;

        if logger.isEnabledFor(logging.DEBUG):
            logger.debug(f'a_enu\n{self._a_enu}')

    def _coordinates(self):
        v_e = self._v_enu.E
        v_n = self._v_enu.N
        self._coord.phi = self._coord.phi + (v_n/(self.R+self._H))*self.dt
        self._coord.lambd = self._coord.lambd + (v_e/((self.R+self._H) * cos(self._coord.phi))) * self.dt
        if logger.isEnabledFor(logging.DEBUG):
            logger.debug(f'phi,lambda\n{self._coord}')

    def _ang_velocity_body_enu(self):

        v = self._v_enu; r = self.R; h = self._H; c = self._coord

        self._w_enu.E = v.N/(r+h) # wox <=> we
        self._w_enu.N = v.E/(r+h) + self.U * cos(c.phi) # woy <=> wn
        self._w_enu.U = v.E/(r+h)*tan(c.phi) + self.U * sin(c.phi) # woz <=> wup

        if logger.isEnabledFor(logging.DEBUG):
            logger.debug(f'w_enu\n{self._w_enu}')
    
    def _speed(self):
        w = self._w_enu; a = self._a_enu; c = self._coord; v = self._v_enu; U = self.U

        self._v_enu.E =  v.E + (a.E + (U*sin(c.phi)+w.Up)*v.N - v.Up*(U*cos(c.phi)+w.E))*self.dt
        self._v_enu.N =  v.N + (a.N - (U*sin(c.phi)+w.Up)*v.E - v.Up*w.E)*self.dt
        # v_up. Unstable channel cant be calculated, so assuming 0
        self._v_enu.Up = 0
        #self._v_enu[2] = v[2] + (a[2] + (self.U*cos(coord[1])+w[1])*v[0] - v[1]*w[0] - 9.81)*self.dt

        if logger.isEnabledFor(logging.DEBUG):
            logger.debug(f'v_enu\n{self._v_enu}')

    def iter_once(self):
        # calculate values. One iteration
        self._acc_body_enu()
        self._speed()
        self._ang_velocity_body_enu()
        self._puasson_equation()
        self._euler_angles()
        self._coordinates()

    def rph_from_acc(self, ax_b:ndarray, ay_b:ndarray, az_b:ndarray):
        ax_mean = np.mean(ax_b); ay_mean = np.mean(ay_b); az_mean = np.mean(az_b)

        st = ay_mean/self.G; sg = -1*ax_mean/math.sqrt(ax_mean**2 + az_mean**2)
        ct = math.sqrt(ax_mean**2 + az_mean**2)/self.G;
        cg = az_mean/math.sqrt(ax_mean**2 + az_mean**2)

        teta = np.arctan2(st, ct)
        gamma = np.arctan2(sg, cg)
        return teta,gamma

    def alignment(self, roll, pitch, heading):
        # psi
        sp = np.sin(heading); cp = np.cos(heading)
        # teta
        st = np.sin(pitch); ct = np.cos(pitch)
        # gamma
        sg = np. sin(roll); cg = np.cos(roll)

        a11 = cp*cg + sp*st*sg
        a12 = sp*ct
        a13 = cp*sg - sp*st*cg
        a21 = -sp*cg + cp*st*sg
        a22 = cp*ct
        a23 = -sp*sg - cp*st*cg
        a31 = -ct*sg
        a32 = st
        a33 = ct*cg

        C_body_enu = np.array([
            [a11, a12, a13],
            [a21, a22, a23],
            [a31, a32, a33]
        ])

        self._tm_body_enu = C_body_enu
    
    '''
    def hw_pre(self, heading, roll, pitch):
        psi = math.radians(heading)
        teta= math.radians(pitch);
        gamma= math.radians(roll);

        sp = np.sin(psi)
        st = np.sin(teta)
        sg = np.sin(gamma)

        cp = np.cos(psi)
        ct = np.cos(teta)
        cg = np.cos(gamma)

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
        C_o_body= np.array([
            [a11, a12, a13],
            [a21, a22, a23],
            [a31, a32, a33]
        ])

        # enu to body matrix
        return C_o_body,C_o_body.transpose()

    def alignment(self, based_on_real_data=False, heading=0, alignment_time=60, pitch=0, roll=0):
        """
        """

        self.alignment_time = alignment_time


        C_body_enu:ndarray
        C_enu_body:ndarray

        a_enu = self.a_enu;
        w_enu = self._w_enu
        if roll != 0 and pitch != 0:
            C_o_b, C_b_o = self.hw_pre(heading, roll, pitch)
            a_pre = C_o_b @ a_enu
            self.a_pre = a_pre
            self.sensor_data["Acc_X"] = [z+a_pre[0] for z in self.sensor_data["Acc_X"] ]
            self.sensor_data["Acc_Y"] = [z+a_pre[1] for z in self.sensor_data["Acc_Y"] ]
            self.sensor_data["Acc_Z"] = [z+a_pre[2] for z in self.sensor_data["Acc_Z"] ]

        if based_on_real_data:
            if logger.isEnabledFor(logging.INFO):
                logger.info(f'Alignment based on real data')
            psi = math.radians(heading)
            C_enu_body,C_body_enu = self.alignment_matrix(
                self.sensor_data["Acc_X"],
                self.sensor_data["Acc_Y"],
                self.sensor_data["Acc_Z"],
                psi
            )
            self._tm_body_enu = C_body_enu.copy()
        else:
            if logger.isEnabledFor(logging.INFO):
                logger.info(f'Ideal alignment')
            # assuming ideal alignment
            C_body_enu = self._tm_body_enu
            C_enu_body = C_body_enu.transpose()

        # re-project vect
        self.a_after_alignment_body = C_enu_body @ a_enu
        self.w_after_alignment_body = C_enu_body @ w_enu
        self.is_aligned = True

        if logger.isEnabledFor(logging.DEBUG):
            logger.debug(f'a_body: {self.a_after_alignment_body}\n')
            logger.debug(f'w_body: {self.w_after_alignment_body}\n')
            logger.debug(f'C_body_enu: {self._tm_body_enu}')
    '''