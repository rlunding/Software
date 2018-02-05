import math

class PI_controller():
    def __init__(self):
        self.d_err = 0
        self.d_integral = 0

        # Gains
        self.k_d = -25
        self.k_Id = 0

# Inputs:   d_est   Estimation of distance from lane center (positve when
#                   offset to the left of driving direction) [m]
#           phi_est Estimation of angle of bot (positive when angle to the
#                   left of driving direction) [rad]
#           d_ref   Reference of d (for lane following, d_ref = 0) [m]
#           v_ref   Reference of velocity [m/s]
#           t_delay Delay it took from taking image up to now [s]
#           dt_last Time it took from last processing to current [s]

# Output:   v_out       velocity of Duckiebot [gain, element of [0,1]]
#           omega_out   angular velocity of Duckiebot []

    def getControlOutput(self, d_est, phi_est, d_ref, v_ref, t_delay, dt_last):

        # obtain last error
        prev_d_err = self.d_err

        # Calculate new error
        self.d_err = d_est - d_ref

        # Integration
        if dt_last is not None:
            self.d_integral += self.d_err * dt_last

        # Saturation
        if math.fabs(self.d_integral) > 0.3:
            self.d_integral = math.copysign(0.3, self.d_integral)


        # Resetting the integral term to avoid oszillations
        if abs(self.d_err) <= 0.011:
            self.d_integral = 0
        # And filter out noise
        if self.sign(self.d_err) != self.sign(prev_d_err):
            self.d_integral = 0


        # Applying controller
        omega =  self.k_d * self.d_err
        omega -= self.k_Id * self.d_integral

        # Declaring return values
        omega_out = omega
        v_out = v_ref
        return (v_out, omega_out)



    def sign(self, x):
        if x == 0:
            return 0
        if x > 0:
            return 1
        else:
            return -1
