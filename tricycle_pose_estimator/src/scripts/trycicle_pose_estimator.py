"""
#########Brain Corp. coding test overview

This code is based on the well-known sample code of Extended kalman filter (EKF) from Atsushi Sakai (@Atsushi_twi). The sample code was modified for the coding test of Brain Corporation.

My solution is based on the EKF due to the non-linearity of the system. The kinematic equations of the tricycle motion model were taken from http://www.moodle2.tfe.umu.se/pluginfile.php/52988/mod_resource/content/2/Kinematics.pdf

Github of the sample code: https://github.com/AtsushiSakai/PythonRobotics

The code was tested with the dataset provided in the coding test zip file. Charts of of each dataset are included in the solution code zip file.

#########Instructions to run the python script:

In order to run the python script, execute it on the terminal in the following way:

python trycicle_pose_estimator.py name_of_the_input_file.csv

. Please provide the path of the input file as an argument to the python script. The input file extension has to be cvs. Note: This code only runs on python2.7 interpreter and it requieres to have you install numpy.

Author: Andres Torres Garcia

"""
# coding=utf-8

import numpy as np
import math
import csv
import matplotlib.pyplot as plt
import sys

class Tricycle(object):
    """docstring for Tricycle"""
    def __init__(self):

        self.counter = 0

        self.n = 7 # States
        self.m = 4 # Observations

        # State Vector [x y yaw v w vs alpha]'
        self.x_est = np.zeros((self.n, 1))
        self.p_est = np.zeros((self.n, self.n))
        self.prev_time = 0
        self.prev_ticks = 0

        # Estimation parameter of EKF
        self.r = np.identity(self.n)
        self.q = np.identity(self.m)

        self.steering_angle_limit = np.pi / 2.0 # Defines +/- steering angle limit in radians
        self.r_dist = 1.0       # Distance b/w front and rear axis (meters)
        self.d_dist = 0.75      # Distance b/w rear wheels (meters)
        self.rear_radius = 0.2  # (meters)
        self.front_radius = 0.2 # (meters)
        self.ticks_per_rev = 512 # unit-less
        self.ticks_per_meter = (2 * np.pi * self.front_radius) / self.ticks_per_rev

    def set_r(self, r):
        if r.shape == self.r.shape:
            self.r = r
        else:
            print("ERROR: The Process Noise Covariance Matrix [q] must be of size [n x n]")

    def set_q(self, q):
        if q.shape == self.q.shape:
            self.q = q
        else:
            print("ERROR: The Measurement Error Covariance Matrix [r] must be of size [m x m]")

    def jacob_f(self, x, dt):
        """
        :brief: Update the state-transition model

            f(x,t) = |   x(t-1) + v(t-1)dt*cos(θ(t-1))  |
                     |   y(t-1) + v(t-1)dt*sin(θ(t-1))  |
                     |        θ(t-1) + ω(t-1)dt         |
                     |       vs(t-1)*cos(α(t-1))        |
                     |       vs(t-1)*sin(α(t-1)) / L    |
                     |              vs(t-1)             |
                     |              α(t-1)              |

        :param dt:
            Change in time [seconds] since last model update
        :return Fx:
            The updated model's state-transition matrix Jacobian
        """
        fx = np.array([ [1, 0, -dt*np.sin(x[2, 0]), dt*np.cos(x[2, 0]), 0, 0, 0],
                        [0, 1, -dt*np.cos(x[2, 0]), dt*np.sin(x[2, 0]), 0, 0, 0],
                        [0, 0, 1, 0, dt, 0, 0],
                        [0, 0, 0, 0, 0, np.cos(x[6, 0]), 0],
                        [0, 0, 0, 0, 0, np.sin(x[6, 0])/self.d_dist, 0],
                        [0, 0, 0, 0, 0, 1, 0],
                        [0, 0, 0, 0, 0, 0, 1] ])

        return fx

    def jacob_h(self, x):
        hx = np.array([  [0, 0, 0, 0, 0, 0, 1],
                         [0, 0, 0, 0, 0, 1, 0],
                         [0, 0, 0, 0, 0, np.sin(x[6, 0])/self.d_dist, 0],
                         [0, 0, 0, 0, 1, 0, 0] ])
        return hx

    def ekf_estimation( self, x_est, p_est, z, dt, debug=False):

        state_noise = np.random.normal(np.array([[0.000001],[0.0001],[0.001],[0.0001],[0.0001],[0.0001],[0.00001]]),scale=0.1)
        obs_noise = np.random.normal(np.array([[0.0],[0.0],[0.0],[0.0]]),scale=0.10)

        """
        Prediction Stage
            xhat = F * xhat + wk
            Phat = F * Phat * trans(F) + R
        """

        #  Predict
        jf = self.jacob_f( x_est, dt ) # [ n * n ]
        x_pred = jf.dot( x_est ) # [ n * 1 ]
        p_pred = jf.dot( p_est ).dot( jf.T ) + self.r # [ n * n ]

        if debug == True:
            print(" angle:\r\n" + str(x_est[2]) + " " + str(x_est[6]) + "\r\n")
            print("Matrices:")
            print(" Inputs:\r\n" + str(z) + "\r\n")
            print(" Old x:\r\n" + str(x_est) + "\r\n")
            print(" Old P:\r\n" + str(p_est) + "\r\n")
            print(" F:\r\n" + str(jf) + "\r\n")
            print(" Predicted X:\r\n" + str(x_pred) + "\r\n")
            print(" Predicted P:\r\n" + str(p_pred) + "\r\n")

        """
        Update Stage
            residual = data - H * xhat            # Measurement Residual
            S = H * Phat * trans(H) + Q           # Update Innovation Covariance
            K = Phat * trans(H) * inv(S)          # Kalman Gain

            xhat = xhat + K * residual;
            Phat = (In - K * H) * Phat;
        """

        #  Update
        j_h = self.jacob_h( x_est ) # [ m * n ]
        # j_h = self.jacob_h( x_pred ) # [ m * n ]
        z_pred = j_h.dot( x_pred ) # [ m * 1 ]
        y = z - z_pred # [ m * 1 ]
        s = j_h.dot( p_pred ).dot( j_h.T ) + self.q # [ m * m ]
        k = p_pred.dot( j_h.T ).dot( np.linalg.inv( s ) ) # [ n * m ]
        x_est = x_pred + k.dot( y ) # [ n * 1 ]
        p_est = ( np.eye( len( x_est ) ) - k.dot( j_h ) ).dot( p_pred ) # [ n * n ]

        if debug == True:
            print(" Jacobian H:\r\n" + str(j_h) + "\r\n")
            print(" Predicted Z:\r\n" + str(z_pred) + "\r\n")
            print(" Z:\r\n" + str(z) + "\r\n")
            print(" Residual:\r\n" + str(y) + "\r\n")
            print(" Residual Covariance:\r\n" + str(s) + "\r\n")
            print(" k:\r\n" + str(k) + "\r\n")
            print(" Updated States [X]:\r\n" + str(x_est) + "\r\n")
            print(" New P:\r\n" + str(p_est) + "\r\n")

        return x_est, p_est

    def estimate( self, time, steering_angle, encoder_ticks, angular_velocity, debug=False ):

        if debug == True:
            print("--------- Estimate Update #" + str(self.counter) + " " + str(time) + " ---------")

        if self.counter == 0:
            self.prev_time = time
            self.counter += 1
            self.prev_ticks = encoder_ticks
            if debug == True:
                print("---- ----    Skipping first data entry with Time = " + str(time))
            return self.x_est[ :3, 0 ]

        dt = time - self.prev_time

        if debug == True:
            print(" dt = {} - {} = {} ".format(time, self.prev_time, dt))

        # Calculate the incremental encoder difference (ticks) since last step
        encod_diff = encoder_ticks - self.prev_ticks
        # Calculate the linear displacement (meters) since last step, based on change in encoder rotation
        displacement_step = encod_diff * self.ticks_per_meter
        # Convert encoder angular displacement into velocity for compatibility with state-transition models
        encoder_vel = displacement_step / dt
        angular_velocity_imu = encoder_vel * np.sin( steering_angle ) / self.d_dist

        z = np.array( [ [ steering_angle ], [encoder_vel], [angular_velocity_imu], [angular_velocity] ] )

        self.x_est, self.p_est = self.ekf_estimation(self.x_est, self.p_est, z, dt)

        self.counter += 1
        self.prev_ticks = encoder_ticks
        self.prev_time = time

        if debug == True:
            print(" Estimation Used Inputs: " + str(z))
            print("\n")
            print(" Estimated Pose: " + str(self.x_est))

        return self.x_est[ :3, 0 ]

    def inputs_prepro( self, input ):

        raw_inputs = []

        #time(sec), encoder(ticks), angular velocity(rad/s), steering angle(rad) -> brain corp
        time = input[ 0 ]
        encoder_ticks = input[ 1 ]
        angular_velocity = input[ 2 ]
        steering_angle = input[ 3 ]

        # Check if steering angle is outside our working limits regardless of input (Safety)
        if steering_angle < -self.steering_angle_limit:
            steering_angle = -self.steering_angle_limit
        elif steering_angle > self.steering_angle_limit:
            steering_angle = self.steering_angle_limit

        return time, steering_angle, encoder_ticks, angular_velocity, raw_inputs

def main():
    print(__file__ + " start!!")

    if len(sys.argv) !=2:
        print("Need input file name (e.g., trycile_pose_estimator.py dataset0.csv)")
        sys.exit()

    file = sys.argv[1]
    output = sys.argv[1][:-4]
    np.set_printoptions(suppress=True)
    np.set_printoptions(precision=4)

    inputs = np.genfromtxt(file, delimiter=',')
    inputs_size = np.size( inputs, 0 )
    show_animation = False

    tricyle = Tricycle()

    # Tune EKF Parameters before attaching EKF to model
    test_r = np.array([0.5, 0.5, 0.01, 1.0, 0.01, 1.0, 1.0])
    test_q = np.array([1.0, 0.7, 1.0, 0.9])
    tricyle.set_r(np.diag(test_r))
    tricyle.set_q(np.diag(test_q))

    debug = False

    # x, y, heading
    _est_poses = []

    for i in range( 0, inputs_size ):
        
        [ time, steering_angle, encoder_ticks, angular_velocity, raw_inputs ] = tricyle.inputs_prepro( inputs[ i, : ] )

        if debug == True:
            print("\n Raw Inputs: " + str(raw_inputs))

        fixed_inputs = [steering_angle, encoder_ticks, angular_velocity]

        if debug == True:
            print(" Filtered Inputs: " + str(fixed_inputs))
            print("\n")

        estimated_pose = tricyle.estimate( time, steering_angle, encoder_ticks, angular_velocity )

        estimated_pose = np.array( estimated_pose ).reshape( 3, 1 )
        tmp = [ estimated_pose[ 0, 0 ], estimated_pose[ 1, 0 ], estimated_pose[ 2, 0 ] ]
        _est_poses.append( tmp )

    est_poses = np.array(_est_poses)

    # Save results to file for post-analysis
    np.savetxt(output + '_output.csv', est_poses, delimiter=',',header='predX, predY, predHeading')

if __name__ == '__main__':
    main()