
import sys
import numpy as np
import matplotlib.pyplot as plt
import json
from time import sleep
from os import path, getenv
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python/")
sys.path.append(PPRZ_SRC + "/sw/lib/python")
from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage
from settings_xml_parse import PaparazziACSettings

class Aircraft(object):
    def __init__(self, ac_id, freq=100.):
        self.id = ac_id
        self.pos = np.array([[0.0],[0.0],[0.0]])
        self.attitude = np.array([[0.0],[0.0],[0.0]])
        self.vel = np.array([[0.0],[0.0],[0.0]])
        self.step = 1. / freq
        self._interface = IvyMessagesInterface("Controller 1")

        self.target_pos = np.array([[5.0],[7.0],[10.0]])
        self.target_vel = np.array([[0.0],[0.0],[0.0]])
        self.error_pos = np.array([[0.0],[0.0],[0.0]])
        self.error_vel = np.array([[0.0],[0.0],[0.0]])
        self.U = np.array([[0.0],[0.0]])   # Accelerations

        # Circle
        self.e = 0.0
        self.grad = np.array([[0.0],[0.0]])
        self.hess = np.array([[2.0,0.0],[0.0,2.0]])
        self.rot = np.array([[0.0,-1.0],[1.0,0.0]])

        self.vel_d = np.array([[0.0],[0.0]])
        self.s = 1.

        self.c1 = 0.005
        self.c2 = 7.
        self.c3 = 1.25

	
        
        
        def rotorcraft_fp_callback(_id, msg):
            if msg.name == "ROTORCRAFT_FP":
                field_coefs = msg.fieldcoefs

                self.pos[0][0] = float(msg.get_field(0)) * field_coefs[0]
                self.pos[1][0] = float(msg.get_field(1)) * field_coefs[1]
                self.pos[2][0] = float(msg.get_field(2)) * field_coefs[2]

                self.vel[0][0] = float(msg.get_field(3)) * field_coefs[3]
                self.vel[1][0] = float(msg.get_field(4)) * field_coefs[4]
                self.vel[2][0] = float(msg.get_field(5)) * field_coefs[5]

                self.attitude[0][0] = float(msg.get_field(6)) * field_coefs[6]
                self.attitude[1][0] = float(msg.get_field(7)) * field_coefs[7]
                self.attitude[2][0] = float(msg.get_field(8)) * field_coefs[8]
        self._interface.subscribe(rotorcraft_fp_callback, PprzMessage("telemetry", "ROTORCRAFT_FP"))

    def print(self):
        print("\n\nID: \t"  + str(self.id) + 
        "\n\neast: \t"      + str(self.pos[0][0]) + 
        "\nnorth: \t"       + str(self.pos[1][0]) + 
        "\nup: \t"          + str(self.pos[2][0]) + 
        "\n\nRoll: \t"      + str(self.attitude[0][0]) + 
        "\nPitch: \t"       + str(self.attitude[1][0]) + 
        "\nYaw: \t"         + str(self.attitude[2][0]) +
        "\nU: \t"         + str(np.linalg.norm(self.U[0:2])) +
        "\nerror: \t"         + str(self.U))


    def send_accelerations(self):
        msg = PprzMessage("datalink", "DESIRED_SETPOINT")
        msg['ac_id'] = self.id
        msg['flag'] = 0 # full 3D not supported yet
        # Receiving position in ENU, but sending in NED:
        msg['uy'] = self.U[0][0]
        msg['ux'] = self.U[1][0]
        msg['uz'] = 20	#self.U[2][0]     # Sending altitude, not acceleration
        self._interface.send(msg)

    def stop(self):
        # Stop IVY interface
        if self._interface is not None:
            self._interface.shutdown()

    def run(self):
        try:
            # The main loop
            while True:
                #sleep(self.step)

                F = -self.c1 * self.e * self.grad
                L = -self.c2 * (self.vel[0:2] - self.vel_d)
                origin = [0], [0] # origin point
                plt.cla()
                plt.quiver(*origin, self.vel_d[0], self.vel_d[1], color=['r'], angles='xy', scale_units='xy', scale=1)
                plt.quiver(*origin, F[0], F[1], color=['b'], angles='xy', scale_units='xy', scale=1)
                plt.quiver(*origin, L[0], L[1], color=['g'], angles='xy', scale_units='xy', scale=1)
                #plt.quiver(*origin, self.vel[0], self.vel[1], color=['b'], angles='xy', scale_units='xy', scale=1)
                #plt.quiver(*origin, self.U[0], self.U[1], color=['g'], angles='xy', scale_units='xy', scale=100)
                #plt.autoscale(enable=True, axis='both', tight=None)
                plt.xlim(-10, 10)
                plt.ylim(-10, 10)
                
                plt.pause(self.step)

                self.circle_path()
                self.calculate_accelerations_path()
                self.send_accelerations()
                #self.print()

        except KeyboardInterrupt:
            self.stop()


    def calculate_accelerations_path(self):
        xt = self.rot.dot(self.grad)
        xt_dot = self.rot.dot(self.hess).dot(self.vel[0:2])
        xt_norm = np.where( np.linalg.norm(xt) != 0., np.linalg.norm(xt), 0.00001 )
        self.vel_d = self.s * xt / xt_norm
        xt_dot_xt = np.where( np.transpose(xt).dot(xt) != 0., np.transpose(xt).dot(xt)**(-3./2.), 1. )
        acc_d = xt_dot / xt_norm + xt * (-np.transpose(xt).dot(xt_dot)) * xt_dot_xt
        self.U = -self.c1 * self.e * self.grad - self.c2 * (self.vel[0:2] - self.vel_d) + self.c3*acc_d


    def circle_path(self):
        radius = 10.
        center = np.array([[0.0],[0.0]])
        self.e = (self.pos[0][0] - center[0][0])**2 + (self.pos[1][0] - center[1][0])**2 - radius**2
        self.grad[0][0] = 2.0 * (self.pos[0][0] - center[0][0])
        self.grad[1][0] = 2.0 * (self.pos[1][0] - center[1][0])
        

if __name__ == '__main__':

    fc = Aircraft(203)
    fc.run()
