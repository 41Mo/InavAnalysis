class vec_body:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

class vec_enu:
    def __init__(self, E=0, N=0, Up=0):
        self.E = E
        self.N = N
        self.Up = Up

class vec_coordinates:
    def __init__(self, lambd=0, phi=0):
        self.lambd = lambd
        self.phi = phi
    def get_lat(self):
        return self.phi
    def get_lon(self):
        return self.lambd
class vec_orientation:
    def __init__(self, gamma=0, teta=0, psi=0 ):
        self.psi = psi
        self.teta = teta
        self.gamma = gamma
    def get_roll(self):
        return self.gamma
    def get_pitch(self):
        return self.teta
    def get_heading(self):
        return self.psi