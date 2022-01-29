import nlopt
import numpy as np
import numdifftools as nd
from scipy.sparse import coo_matrix

# Set the timestep length and duration
N = 2   # prediction Horizon
dt = 0.08
T = N * dt # This is the Prediction Horizon in seconds.


'''
This solver takes all the state variables and actuator variables in a single vector.
Here, we establish when one variable starts and another ends to be able to address its
indexes in an easy way.
'''

# State
x_start = 0
y_start = x_start + N
yaw_start = y_start + N
v_start = yaw_start + N
cte_start = v_start + N
yaw_err_start = cte_start + N
speed_err_start = yaw_err_start + N

# Outputs (First)
steer_start = speed_err_start + N - 1
throttle_start = steer_start + N - 1
brake_start = throttle_start + N - 1


'''
define the WEIGHTS that we will use to quantify how "costly" (bad) are each component
of the COST function. Basically, HOW imnportant is each element of the COST function:
    For instance, it's very important that cte remains close to 0 but also it's very
    important to make sure that the changes in commands (steering and throttle) are smooth.
'''

W_cte = 1
W_yaw_err = 1
W_vel_err = 9
W_steer_use = 0
W_throttle_use = 0
W_brake_use = 0
W_dSteer = 9    # Differential Steering
W_dThrottle = 0 # Differential Throttle
W_dbrake = 0    # Differential Brake



'''
The constructor of the ipopt.problem class requires:
    n : the number of variables in the problem,
    m : the number of constraints in the problem,
    lb and ub : lower and upper bounds on the variable, adn
    cl and cu : lower and upper bounds of the constraints.
    problem_obj is an object whose methods implement the objective, gradient, constraints,
                jacobian and hessian of the problem
'''

class MPC(object):
    def __init__(self):
        self.speed_ref = 0
        self.coeffs = []
        self.n_constraints = 0

    def Solve(self, state, coeffs):
        self.coeffs = coeffs

        ok = True

        num_state_variables = len(state)
        num_outputs = 3

        '''
        Set the number of model variables (includes both states and inputs).
        For example :: If the state is a 4 element vector and the actuators is
                       a 2 element vector and there are 10 timesteps.
                       The number of variables is :
                       4 * 10 + 2 * 9
                       In "N" timesteps => "N - 1" actuations
        '''

        n_vars = N * num_state_variables + (N-1) * num_outputs

        # Set the number of constraints over the State Variables
        self.n_constraints = N * (num_state_variables)


        # For clarity
        x = state[0]    # Always 0 since we moved to the Car Ref System
        y = state[1]    # Always 0 since we moved to the Car Ref System
        yaw = state[2]  # Always 0 since we moved to the Car Ref System
        v = state[3]
        cte = state[4]
        yaw_err = state[5]
        speed_err = state[6]
        self.speed_ref = speed_err + v

        # Initial value of the independent variable
            # SHOULD BE 0 besides initial state
        # Initial State :
            # Set the initial variable values
        vars = np.zeros(n_vars)

        vars[x_start] = x
        vars[y_start] = y
        vars[yaw_start] = yaw
        vars[v_start] = v
        vars[cte_start] = cte
        vars[yaw_err_start] = yaw_err
        vars[speed_err_start] = speed_err

        vars_lowerbound  = np.zeros(n_vars)
        vars_upperbounnd = np.zeros(n_vars)


        '''
        -> Set lower and upper limits for variables
        -> Set all non-actuators (x,y,yaw,v,cte,yaw_err) upper and lower limits 
            to the max negative and positive values.
        -> We can refine these limits but for simplicity we do this for now.
        '''

        for i in range(0, steer_start):
            vars_lowerbound[i] = -1.0e19
            vars_upperbound[i] = 1.e019

        # The upper and lower limits for Steering is -1.22 to 1.22 Radians
        for i in range(steer_start, throttle_start):
            vars_lowerbound[i] = -1.22
            vars_upperbound[i] = 1.22

        # The upper and lower limits Throttle is 0 to 1. (%)
        for i in range(throttle_start, brake_start):
            vars_lowerbound[i] = 0
            vars_upperbound[i] = 1.0

        # The upper and lower limits for Brake ARE 0 to 1. (%)
        for i in range(brake_start, n_vars):
            vars_lowerbound[i] = 0
            vars_upperbound[i] = 1.0


