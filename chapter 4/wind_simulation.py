import sys
sys.path.append('..')
from transfer_function import transferFunction
import numpy as np
import sphere_parameters as MAV


class WindSimulation:
    def __init__(self, Ts):
        # Steady state wind as defined in the inertial frame
        self._steady_state = np.array([[0., 0., 0.]]).T

        #   Dryden gust model parameters (section 4.4 UAV book)
        Va =  MAV.Va0  
        Lu = 533.   # 200., 533.
        Lv = 533.   # 200., 533.
        Lw = 533.   # 50., 533.
        gust_flag = True
        if gust_flag==True:
            sigma_u = 1.5  # 1.06, 2.12, 1.5, 3.0
            sigma_v = 1.5  # 1.06, 2.12, 1.5, 3.0
            sigma_w = 1.5  # 0.7, 1.4, 1.5, 3.0
        else:
            sigma_u = 0.
            sigma_v = 0.
            sigma_w = 0.

        # Dryden transfer functions
        b0_u = sigma_u*np.sqrt(2*Va/Lu)
        self.u_w = transferFunction(num=np.array([[b0_u]]), den=np.array([[1, Va/Lu]]),Ts=Ts)
        b1_v = sigma_v*np.sqrt(3*Va/Lv)
        b0_v = sigma_v*np.sqrt(3*Va/Lv)*Va/(np.sqrt(3)*Lv)

        a1_v = 2*Va/Lv
        a0_v = (Va/Lv)**2
        self.v_w = transferFunction(num=np.array([[b1_v, b0_v]]), den=np.array([[1, a1_v, a0_v]]),Ts=Ts)
       
        b1_w = sigma_w*np.sqrt(3*Va/Lw)
        b0_w = sigma_w*np.sqrt(3*Va/Lw)*Va/(np.sqrt(3)*Lw)
        
        a1_w = 2*Va/Lw
        a0_w = (Va/Lw)**2
        self.w_w = transferFunction(num=np.array([[b1_w, b0_w]]), den=np.array([[1, a1_w, a0_w]]),Ts=Ts)
        self._Ts = Ts

    def update(self):
        #   The first three elements are the steady state wind in the inertial frame
        #   The second three elements are the gust in the body frame
        gust = np.array([[self.u_w.update(np.random.randn())],
                         [self.v_w.update(np.random.randn())],
                         [self.w_w.update(np.random.randn())]])
        return np.concatenate((self._steady_state, gust))