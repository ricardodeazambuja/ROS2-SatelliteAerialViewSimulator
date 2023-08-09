import numpy as np

try:
    from quad_sim_python import Quadcopter, Controller
except ImportError:
    raise ImportError("You need to install quad_sim_python: 'pip install quad_sim_python'")


class UAVPhysics:
    def __init__(self, z0, Ts=0.005, ctrlType="xyz_pos", orient="ENU", ctrlParams={}):
        # ctrlTypes are xyz_pos, xy_vel_z_pos or xyz_vel
        self.Ti = 0 # init time
        self.Ts = Ts
        self.ctrlType = ctrlType
        ctrl_params = {
                      # Position P gains
                      "Px"    : 2.0, "Py"    : 2.0, "Pz"    : 1.0,

                      # Velocity P-D gains
                      "Pxdot" : 5.0, "Dxdot" : 0.5, "Ixdot" : 5.0,

                      "Pydot" : 5.0, "Dydot" : 0.5, "Iydot" : 5.0,

                      "Pzdot" : 4.0, "Dzdot" : 0.5, "Izdot" : 5.0,

                      # Attitude P gains
                      "Pphi"   : 8.0, "Ptheta" : 8.0, "Ppsi"   : 1.5,

                      # Rate P-D gains
                      "Pp" : 1.5, "Dp" : 0.04,

                      "Pq" : 1.5, "Dq" : 0.04,

                      "Pr" : 1.0, "Dr" : 0.1,

                      # Max Velocities (x,y,z) [m/s]
                      "uMax" : 5.0, "vMax" : 5.0, "wMax" : 5.0,

                      "saturateVel_separately" : True,

                      # Max tilt [degrees]
                      "tiltMax" : 50.0,

                      # Max Rate [degrees/s]
                      "pMax" : 200.0, "qMax" : 200.0, "rMax" : 150.0,

                      # Minimum velocity for yaw follow to kick in [m/s]
                      "minTotalVel_YawFollow" : 0.1,

                      "useIntegral" : True    # Include integral gains in linear velocity control
                      }

        for k in ctrlParams:
            ctrl_params[k] = ctrlParams[k]

        # Initialize Quadcopter, Controller, Wind, Result Matrixes
        # ---------------------------
        self.initPose = np.array([0,0,z0,0,0,0]) # x0, y0, z0, phi0, theta0, psi0
        self.initTwist = np.array([0,0,0,0,0,0]) # xdot, ydot, zdot, p, q, r
        self.initStates = np.hstack((self.initPose, self.initTwist))

        self.quad = Quadcopter(self.Ti, self.initStates, orient=orient)
        self.ctrl = Controller(self.quad.params, orient=orient, params=ctrl_params)
        self.t = self.Ti

    def update(self, desPos, desVel, dt):
        for i in range(int(dt/self.Ts)):
            self.ctrl.control(self.Ts, self.ctrlType, None, 
                              desPos, desVel, np.array([0., 0., 0.]), np.array([0., 0., 0.]), 0, 0,
                              self.quad.pos, self.quad.vel, self.quad.vel_dot, 
                              self.quad.quat, self.quad.omega, self.quad.omega_dot, self.quad.psi)
            
            w_cmd = self.ctrl.getMotorSpeeds()

            self.quad.update(self.t, self.Ts, w_cmd, [0,0,0])

            self.t += self.Ts

        return self.quad.pos