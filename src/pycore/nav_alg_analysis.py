import numpy as np
from typing import Tuple
import matplotlib.pyplot as plt
import logging
import nav_alg as nav_alg
logger = logging.getLogger(__name__)

class nav_alg_analysis:
    def __init__(self, frequency=1, time=10800, analysis="static", obj_name="" ):
        na = nav_alg.nav_alg()

        """
            frequency [HZ]
            time [seconds]
        """
        # input
        self.frequency = frequency
        self.dt = 1/frequency
        self.number_of_points = int(time / self.dt)
        self.analysis_type = analysis

        self.__name__ = obj_name

        # output
        self.spd_e = []
        self.spd_n = []
        self.pitch = []
        self.roll = []
        self.yaw = []
        self.lat = []
        self.lon = []
        self.a_e = []
        self.a_n = []
        self.a_u = []
        self.w_e = []
        self.w_n = []
        self.w_u = []

    def static_analysis(self):
        for i in range(self.number_of_points):

            self._w_body[0,0] = self._w_body_input[0,0] + \
                self.w_after_alignment_body[0,0]
            self._w_body[1,0] = self._w_body_input[1,0] + \
                self.w_after_alignment_body[1,0]
            self._w_body[2,0] = self._w_body_input[2,0] + \
                self.w_after_alignment_body[2,0]
                           
            self._a_body[0,0] = self._a_body_input[0,0] + \
                self.a_after_alignment_body[0,0]
            self._a_body[1,0] = self._a_body_input[1,0] + \
                self.a_after_alignment_body[1,0]
            self._a_body[2,0] = self._a_body_input[2,0] + \
                self.a_after_alignment_body[2,0]

            if logger.isEnabledFor(logging.DEBUG):
                logger.debug(f'Iteration: {i}')
                logger.debug(f'w_body\n{self._w_body}')
                logger.debug(f'a_body\n{self._a_body}')

            self.calc_and_save()
        self.prepare_data()

    def dynamic_analysis_both(self):
        for i in range(0,self.number_of_points):
            self._w_body[0,0] = self.sensor_data["Gyr_X"][i] + \
                self.w_after_alignment_body[0,0]
            self._w_body[1,0] = self.sensor_data["Gyr_Y"][i] + \
                self.w_after_alignment_body[1,0]
            self._w_body[2,0] = self.sensor_data["Gyr_Z"][i] + \
                self.w_after_alignment_body[2,0]
                           
            self._a_body[0,0] = self.sensor_data["Gyr_X"][i] + \
                self.a_after_alignment_body[0,0]
            self._a_body[1,0] = self.sensor_data["Gyr_Y"][i] + \
                self.a_after_alignment_body[1,0]
            self._a_body[2,0] = self.sensor_data["Gyr_Z"][i] + \
                self.a_after_alignment_body[2,0]
            if logger.isEnabledFor(logging.DEBUG):
                logger.debug(f'Iteration: {i}')
                logger.debug(f'w_body\n{self._w_body}')
                logger.debug(f'a_body\n{self._a_body}')
            
            self.calc_and_save()
        self.prepare_data()
    
    def dynamic_analysis_gyro(self):
        for i in range(0,self.number_of_points):
            self._w_body[0,0] = self.sensor_data["Gyr_X"][i] + \
                self.w_after_alignment_body[0,0]
            self._w_body[1,0] = self.sensor_data["Gyr_Y"][i] + \
                self.w_after_alignment_body[1,0]
            self._w_body[2,0] = self.sensor_data["Gyr_Z"][i] + \
                self.w_after_alignment_body[2,0]

            self._a_body[0,0] =self.a_after_alignment_body[0,0]
            self._a_body[1,0] = self.a_after_alignment_body[1,0]
            self._a_body[2,0] = self.a_after_alignment_body[2,0]

            if logger.isEnabledFor(logging.DEBUG):
                logger.debug(f'Iteration: {i}')
                logger.debug(f'w_body\n{self._w_body}')
                logger.debug(f'a_body\n{self._a_body}')

            self.calc_and_save()
        self.prepare_data()

    def dynamic_analysis_acc(self):
        for i in range(self.starting_point,self.number_of_points):
            self._a_body[0,0] = self.sensor_data["Acc_X"][i] + \
                self.a_after_alignment_body[0,0]
            self._a_body[1,0] = self.sensor_data["Acc_Y"][i] + \
                self.a_after_alignment_body[1,0]
            self._a_body[2,0] = self.sensor_data["Acc_Z"][i] + \
                self.a_after_alignment_body[2,0]

            self._w_body[0,0] = self.w_after_alignment_body[0,0]
            self._w_body[1,0] = self.w_after_alignment_body[1,0]
            self._w_body[2,0] = self.w_after_alignment_body[2,0]

            if logger.isEnabledFor(logging.DEBUG):
                logger.debug(f'Iteration: {i}')
                logger.debug(f'w_body\n{self._w_body}')
                logger.debug(f'a_body\n{self._a_body}')

            self.calc_and_save()
        self.prepare_data()

    def analysis(self): 
        """
            run analysis
        """
        if not self.is_aligned:
            if logger.isEnabledFor(logging.INFO):
                logger.info("Alignment start")
            self.alignment()

        if logger.isEnabledFor(logging.INFO):
            logger.info(f"Initial coordinates. lat: {self._coord[0]}, lon: {self._coord[1]}")

        if self.analysis_type == "static":
            self.static_analysis()
        
        if self.analysis_type == "dynamic_both":
            self.dynamic_analysis_both()

        if self.analysis_type == "dynamic_gyro":
            self.dynamic_analysis_gyro()

        if self.analysis_type == "dynamic_acc":
            self.dynamic_analysis_acc()

    def plots(self, size:Tuple=(140,170), save:bool=False, title:str="", additional_plots:bool=False):
        """
        generate 1 plot with 7 subplots
        - orientation angles
        - speed
        - coordinates
        additional debug plots:
        - a_e, a_n, a_up
        - w_e, w_n, w_up
        """
        #plt.figure(figsize=size)
        #plt.rc('font', size=10) 
        #fig = plt.figure(figsize=size, constrained_layout=True)
        #axs = plt.subplots(7,1)

        size = (size[0]/25.4, size[1]/25.4)

        # setting title with obj_name if title is not defined
        if title == "" and self.__name__ != "":
            title = self.__name__

        fig,axs = plt.subplots(3,1,sharex=True,constrained_layout=True)
        fig.set_size_inches(size)
        #fig.suptitle(title)

        axs[0].plot(np.linspace(0, len(self.pitch)*self.dt, len(self.pitch)), np.rad2deg(self.pitch)*60, label="roll")
        axs[0].set_ylabel('$\\theta$, угл мин')
        axs[1].plot(np.linspace(0, len(self.roll)*self.dt, len(self.roll)), np.rad2deg(self.roll)*60, label="pitch")
        axs[1].set_ylabel('$\gamma$, угл мин')
        axs[2].plot(np.linspace(0, len(self.yaw)*self.dt, len(self.yaw)), np.rad2deg(self.yaw)*60, label="yaw")
        axs[2].set_ylabel('$\psi$, угл мин')
        axs[2].set_xlabel("время, с")

        if save:
            plt.savefig("./images/"+"angles"+title+".jpg", bbox_inches='tight')
        plt.show()

        fig,axs = plt.subplots(2,1,sharex=True,constrained_layout=True)
        fig.set_size_inches(size)

        axs[0].plot(np.linspace(0, len(self.spd_e)*self.dt, len(self.spd_e)), self.spd_e, label="v_e")
        axs[0].set_ylabel('$V_E$, м/c')
        axs[1].plot(np.linspace(0, len(self.spd_n)*self.dt, len(self.spd_n)), self.spd_n, label="v_n")
        axs[1].set_ylabel('$V_N$, м/c')
        axs[1].set_xlabel("время, с")

        if save:
            plt.savefig("./images/"+"speed"+title+".jpg", bbox_inches='tight')
        plt.show()

        fig,axs = plt.subplots(2,1,sharex=True,constrained_layout=True)
        fig.set_size_inches(size)

        axs[0].plot(np.linspace(0, len(self.lat)*self.dt, len(self.lat)), np.rad2deg(self.lat)*111138.5, label="lat")
        axs[0].set_ylabel('$\\varphi$, м')
        axs[1].plot(np.linspace(0, len(self.lon)*self.dt, len(self.lon)), np.rad2deg(self.lon)*111138.5, label="lon")
        axs[1].set_ylabel('$\lambda$, м')
        axs[1].set_xlabel("время, с")


        if save:
            plt.savefig("./images/"+"coord"+title+".jpg", bbox_inches='tight')
        plt.show()

        # additional
        if additional_plots:
            fig,axs = plt.subplots(6,1,sharex=True,constrained_layout=True)
            fig.set_size_inches(size)
            fig.suptitle(title)

            x_time = np.linspace(0, len(self.w_e)*self.dt, len(self.w_e))
            axs[0].plot(x_time, np.rad2deg(self.w_e))
            axs[0].set_ylabel('w_e')

            axs[1].plot(x_time, np.rad2deg(self.w_n))
            axs[1].set_ylabel('w_n')

            axs[2].plot(x_time, np.rad2deg(self.w_u))
            axs[2].set_ylabel('w_u')

            axs[3].plot(x_time, (self.a_e))
            axs[3].set_ylabel('a_e')

            axs[4].plot(x_time, (self.a_n))
            axs[4].set_ylabel('a_n')

            axs[5].plot(x_time, (self.a_u))
            axs[5].set_ylabel('a_u')
            axs[5].set_xlabel("время, с")
            plt.show()


            if self.analysis_type == "dynamic_gyro" :

                fig,axs = plt.subplots(3,1,sharex=True,constrained_layout=True)
                fig.set_size_inches(size)
                fig.suptitle(title)

                x_time = np.linspace(0, len(self.sensor_data["Gyr_X"])*self.dt, len(self.sensor_data["Gyr_X"]))

                axs[0].plot(x_time, np.rad2deg(self.sensor_data["Gyr_X"]))
                axs[0].set_ylabel('wx_b')

                axs[1].plot(x_time, np.rad2deg(self.sensor_data["Gyr_Y"]))
                axs[1].set_ylabel('wy_b')

                axs[2].plot(x_time, np.rad2deg(self.sensor_data["Gyr_Z"]))
                axs[2].set_ylabel('wz_b')

                plt.show()

            if self.analysis_type == "dynamic_acc":
                fig,axs = plt.subplots(3,1,sharex=True,constrained_layout=True)
                fig.set_size_inches(size)
                fig.suptitle(title)

                x_time = np.linspace(0, len(self.sensor_data["Acc_X"])*self.dt, len(self.sensor_data["Acc_X"]))
                axs[0].plot(x_time, self.sensor_data["Acc_X"])
                axs[0].set_ylabel('ax_b')

                axs[1].plot(x_time, self.sensor_data["Acc_Y"])
                axs[1].set_ylabel('ay_b')

                axs[2].plot(x_time, self.sensor_data["Acc_Z"])
                axs[2].set_ylabel('az_b')

                plt.show()