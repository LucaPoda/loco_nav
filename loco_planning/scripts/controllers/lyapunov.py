import numpy as np
import math
from utils.math_tools import unwrap_angle
from utils.math_tools import Math

class LyapunovParams:
    def __init__(self, K_P, K_THETA, DT=0.001, ESTIMATE_ALPHA_WITH_ACTUAL_VALUES = False):
        self.K_P = K_P
        self.K_THETA = K_THETA
        self.DT = DT
class Robot:
    pass

class LyapunovController:
    def __init__(self, params: LyapunovParams): #, matlab_engine = None):

        self.K_P = params.K_P
        self.K_THETA = params.K_THETA
        self.log_e_x = []
        self.log_e_y = []
        self.log_e_theta = []

        self.theta_old = 0.
        self.v_old = 0.
        self.omega_old = 0.

        self.params = params
        self.math_utils = Math()

    def getErrors(self):
        return self.log_e_x, self.log_e_y,  self.log_e_theta

    def control_unicycle(self, actual_state, current_time, des_x, des_y, des_theta, v_d, omega_d, traj_finished):
        """
        ritorna i valori di linear e angular velocity
        """
        if traj_finished:
            # save errors for plotting
            self.log_e_x.append(0.0)
            self.log_e_y.append(0.0)
            self.log_e_theta.append(0.0)
            return 0.0, 0.0, 0., 0.

        # --- 1. ERROR DEFINITION (Actual - Desired) ---
        # If your C++ uses this, Python must match it.
        ex = actual_state.x - des_x
        ey = actual_state.y - des_y

        # --- 2. HEADING ERROR ---
        # Using atan2 to handle the +/- pi jumps safely
        theta = actual_state.theta
        des_theta_w = math.atan2(math.sin(des_theta), math.cos(des_theta))
        
        # Error = Actual - Desired
        raw_etheta = theta - des_theta_w
        etheta = (raw_etheta + np.pi) % (2 * np.pi) - np.pi
        
        # Beta is required for the Lyapunov domega term
        beta = theta + des_theta_w

        # --- 3. POLAR COORDINATES ---
        exy = math.sqrt(ex**2 + ey**2)
        psi = math.atan2(ey, ex)

        # --- 4. CONTROL LAW (Note the NEGATIVE signs for stability) ---
        # Since error is (Act - Des), we need negative feedback to reduce it.
        dv = -self.K_P * exy * math.cos(psi - theta)
        
        # Guard the denominator to prevent infinite omega commands
        denom = np.cos(etheta / 2.0)
        if abs(denom) < 0.05: 
            denom = 0.05 * np.sign(denom)

        # domega: Feed-forward term + Heading correction
        # The negative signs ensure V_dot stays <= 0
        domega = -v_d * exy * (1.0 / denom) * np.sin(psi - (beta / 2.0)) - self.K_THETA * np.sin(etheta)

        # Final Commands
        v = v_d + dv
        omega = omega_d + domega

        # --- 4. STABILITY MONITOR (V_dot) ---
        # If this is > 0, the controller is driving AWAY from the target
        v_dot = -self.K_P * (exy**2) * (np.cos(theta - psi)**2) - self.K_THETA * (np.sin(etheta)**2)
        
        if v_dot > 0.001:
            rospy.logwarn_throttle(1, f"Unstable! V_dot: {v_dot:.4f}")

        # --- 5. LOGGING ---
        self.log_e_x.append(ex)
        self.log_e_y.append(ey)
        self.log_e_theta.append(etheta)

        return v, omega

