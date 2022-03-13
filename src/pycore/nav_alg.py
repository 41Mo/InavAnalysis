import numpy as np
from numpy import cos as cos, ndarray, roll
from numpy import sin as sin
from numpy import tan as tan
import math as math
import logging
logger = logging.getLogger(__name__)
class nav_alg:

    # Earth parameters
    R = 6378245.0; # earth radius [m]
    U = math.radians(15)/3600 # earth speed [rad/sec]
    G = 9.81 # [m/s/s]

    def __init__(self):

        # default varaiables
        self._H = 0.0 # object height above ground
        self._w_body = np.array([0, 0, 0], dtype=np.double).reshape(3,1) # angle speed body
        self._a_body = np.array([0, 0, 0], dtype=np.double).reshape(3,1) # acceleration body
        self._v_enu  = np.array([0, 0, 0], dtype=np.double).reshape(3,1)         # linear speed enup
        self._coord   = np.array([0,0], dtype=np.double).reshape(2,1)     # lat, lon (phi, lambda)
        self._tm_body_enu    = np.eye(3, dtype=np.double)                        # transformation matrix body - enup
        self.sensor_data = {}

        # local variables
        self._w_enu          = np.array([0,0,0], dtype=np.double).reshape(3,1)         # angle speed enup
        self._a_enu          = np.array([0,0,0], dtype=np.double).reshape(3,1)         # acceleration enup
        self._rph_angles  = np.array([0,0,0], dtype=np.double).reshape(3,1)         # euler angles
        self.a_after_alignment_body = 0.0
        self.w_after_alignment_body = 0.0
        self.is_aligned = False
        self.is_coordinates_set = False
        self._w_body_input = np.array([0, 0, 0], dtype=np.double).reshape(3,1) # user input
        self._a_body_input = np.array([0, 0, 0], dtype=np.double).reshape(3,1) # user input
        self.starting_point = 0
        self.alignment_time=0
        self.a_pre = 0


        a_enu = np.array([[0],[0],[self.G]])
        w_enu = np.array([
            [0],
            [self.U*math.cos(self._coord[0])],
            [self.U*math.sin(self._coord[0])]
        ]
        )

    def set_a_body(self, ax, ay, az):
        self._a_body_input=np.array([ax, ay, az]).reshape(3,1)
        if logger.isEnabledFor(logging.INFO):
            logger.info(f'Object: {self.__name__}\nA_body setup:\n {self._a_body_input}')

    def set_w_body(self, wx, wy, wz):
        self._w_body_input=np.array([wx, wy, wz]).reshape(3,1)
        if logger.isEnabledFor(logging.INFO):
            logger.info(f'Object: {self.__name__}\nW_body setup:\n {self._w_body_input}')

    def set_coordinates(self, lat, lon):
        self._coord = np.array([np.deg2rad(lat), np.deg2rad(lon)]).reshape(2,1)
        self.is_coordinates_set = True
        if logger.isEnabledFor(logging.INFO):
            logger.info(f'Object: {self.__name__}\nCoordinates setup:\n {self._coord}')

    def _puasson_equation(self):
        wbx = self._w_body[0]
        wby = self._w_body[1]
        wbz = self._w_body[2]

        wox = self._w_enu[0]
        woy = self._w_enu[1]
        woz = self._w_enu[2]
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

        # teta
        self._rph_angles[0] = np.arctan(C[1,2]/C_0)
        # gamma
        self._rph_angles[1] = -np.arctan(C[0,2]/C[2,2])
        # psi
        self._rph_angles[2] = np.arctan(C[1,0]/C[1,1])
        if logger.isEnabledFor(logging.DEBUG):
            logger.debug(f'Roll,Pitch,Heading\n {self._rph_angles}')

    def _acc_body_enu(self):
        # transformation from horisontal vector to veritical vector
        self._a_enu = (self._tm_body_enu@self._a_body)
        if logger.isEnabledFor(logging.DEBUG):
            logger.debug(f'a_enu\n{self._a_enu}')

    def _coordinates(self):
        lin_spd = self._v_enu
        coords = self._coord

        v_e = lin_spd[0]
        v_n = lin_spd[1]

        # phi
        self._coord[0] = coords[0] + (v_n/(self.R+self._H))*self.dt
        # lambda
        self._coord[1] = coords[1] + (v_e/((self.R+self._H) * cos(coords[0]))) * self.dt
        if logger.isEnabledFor(logging.DEBUG):
            logger.debug(f'phi,lambda\n{self._coord}')

    def _ang_velocity_body_enu(self):
        spd = self._v_enu
        coord = self._coord

        self._w_enu[0] = -spd[1]/(self.R+self._H) # wox <=> we
        self._w_enu[1] = spd[0]/(self.R+self._H) + self.U * cos(coord[0]) # woy <=> wn
        self._w_enu[2] = spd[0]/(self.R+self._H)*tan(coord[0]) + self.U * sin(coord[0]) # woz <=> wup
        if logger.isEnabledFor(logging.DEBUG):
            logger.debug(f'w_enu\n{self._w_enu}')
    
    def _speed(self):
        w = self._w_enu
        a = self._a_enu
        coord = self._coord
        v = self._v_enu
        # v_e
        self._v_enu[0] =  v[0] + (a[0] + (self.U*sin(coord[0])+w[2])*v[1] - v[2]*(self.U*cos(coord[0])+w[1]))*self.dt
        # v_n
        self._v_enu[1] =  v[1] + (a[1] - (self.U*sin(coord[0])+w[2])*v[0] - v[2] * w[0])*self.dt
        # v_up. Unstable channel cant be calculated, so assuming 0
        self._v_enu[2] = 0
        #self._v_enu[2] = v[2] + (a[2] + (self.U*cos(coord[1])+w[1])*v[0] - v[1]*w[0] - 9.81)*self.dt
        if logger.isEnabledFor(logging.DEBUG):
            logger.debug(f'v_enu\n{self._v_enu}')

    def calc_output(self):
        # calculate values on each itaration
        self._acc_body_enu()
        self._speed()
        self._ang_velocity_body_enu()
        self._puasson_equation()
        self._euler_angles()
        self._coordinates()

    def calc_and_save(self):
        self.calc_output()

        # store current values
        v_tmp = self._v_enu.copy()
        c_tmp = self._coord.copy()
        ang_tmp = self._rph_angles.copy()
        self.spd_e.append(v_tmp[0])
        self.spd_n.append(v_tmp[1])
        self.lat.append(c_tmp[0])
        self.lon.append(c_tmp[1])
        self.pitch.append(ang_tmp[0])
        self.roll.append(ang_tmp[1])
        self.yaw.append(ang_tmp[2])

        a_tmp = self._a_enu.copy()
        w_tmp = self._w_enu.copy()
        self.a_e.append(a_tmp[0])
        self.a_n.append(a_tmp[1])
        self.a_u.append(a_tmp[2])
        self.w_e.append(w_tmp[0])
        self.w_n.append(w_tmp[1])
        self.w_u.append(w_tmp[2])

    def prepare_data(self):
        self.spd_e.pop(0)
        self.spd_n.pop(0)
        self.lat.pop(0)
        self.lon.pop(0)
        self.pitch.pop(0)
        self.roll.pop(0)
        self.yaw.pop(0)


    def alignment_matrix(self, ax_b:ndarray, ay_b:ndarray, az_b:ndarray, heading):
        slice = self.alignment_time/self.dt
        slice = int(slice)
        self.starting_point = slice
        ax_mean_60_sec = np.mean(ax_b[0:slice])
        ay_mean_60_sec = np.mean(ay_b[0:slice])
        az_mean_60_sec = np.mean(az_b[0:slice])
        psi = heading

        sp = np.sin(psi)
        st = ay_mean_60_sec/self.G
        sg = -1*ax_mean_60_sec/math.sqrt(ax_mean_60_sec**2 + az_mean_60_sec**2)

        cp = np.cos(psi)
        ct = math.sqrt(ax_mean_60_sec**2 + az_mean_60_sec**2)/self.G
        cg = az_mean_60_sec/math.sqrt(ax_mean_60_sec**2 + az_mean_60_sec**2)

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
        C_body_enu = np.array([
            [a11, a12, a13],
            [a21, a22, a23],
            [a31, a32, a33]
        ])

        # enu to body matrix
        return C_body_enu.transpose(),C_body_enu
    
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
