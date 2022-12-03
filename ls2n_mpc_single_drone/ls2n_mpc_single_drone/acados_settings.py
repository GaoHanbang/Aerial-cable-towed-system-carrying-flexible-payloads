from casadi import *
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
import numpy as np
import scipy.linalg
import os
from ament_index_python.packages import get_package_share_directory


def drone_ode_model(mass):

    model_name = 'ls2n_drone'

    m = mass
    g = 9.81 # gravity constant [m/s^2]

    # set up states & controls
    px = SX.sym('px')      # position
    py = SX.sym('py')
    pz = SX.sym('pz')
    qw = SX.sym('qw')      # quaternion
    qx = SX.sym('qx')
    qy = SX.sym('qy')
    qz = SX.sym('qz')
    vx = SX.sym('vx')      # linear velocity in reference frame
    vy = SX.sym('vy')
    vz = SX.sym('vz')
 
    x = vertcat(px, py, pz, qw, qx, qy, qz, vx, vy, vz)

    # controls
    f      = SX.sym('f')       # thrust
    omegax = SX.sym('omegax')  # angular rates
    omegay = SX.sym('omegay')
    omegaz = SX.sym('omegaz')

    u = vertcat(f, omegax, omegay, omegaz)
    
    # xdot
    px_dot = SX.sym('px_dot')
    py_dot = SX.sym('py_dot')
    pz_dot = SX.sym('pz_dot')
    qw_dot = SX.sym('qw_dot')
    qx_dot = SX.sym('qx_dot')
    qy_dot = SX.sym('qy_dot')
    qz_dot = SX.sym('qz_dot')
    vx_dot = SX.sym('vx_dot')
    vy_dot = SX.sym('vy_dot')
    vz_dot = SX.sym('vz_dot')

    xdot = vertcat(px_dot, py_dot, pz_dot, qw_dot, qx_dot, qy_dot, qz_dot, vx_dot, vy_dot, vz_dot)

    # algebraic variables
    # z = None

    # parameters
    p = []
    
    # dynamics
    f_expl = vertcat(
        vx,
        vy,
        vz,
        0.5*(-qx*omegax - qy*omegay - qz*omegaz ), # quaternion derivatives: q_dot = 1/2*q[*]omega, [*]: quaternion multiplication 
        0.5*( qw*omegax + qy*omegaz - qz*omegay ),
        0.5*( qw*omegay + qz*omegax - qx*omegaz ),
        0.5*( qw*omegaz + qx*omegay - qy*omegax ),
        2*(qw*qy + qx*qz)*f/m,                     # acceleration = (1/m)*R*[0; 0; f] + [0; 0; -g], R: rotation matrix (obtained by quaternion)
        2*(qy*qz - qw*qx)*f/m,
        ( 1 - 2*(qx*qx + qy*qy) )*f/m - g
    )

    f_impl = xdot - f_expl

    model = AcadosModel()

    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u
    # model.z = z
    model.p = p
    model.name = model_name

    return model


def ocp_settings(mass, Tf, N, Q, R, x0, max_thrust, max_omega_xy, max_omega_z, max_v):

    # create ocp object
    ocp = AcadosOcp() 
    
    # set model
    model = drone_ode_model(mass) 
    ocp.model = model

    # set dimensions
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu
    ny_e = nx

    ocp.dims.N = N

    # set cost module
    ocp.cost.cost_type = 'LINEAR_LS'
    ocp.cost.cost_type_e = 'LINEAR_LS'

    cost_Q = np.diag(Q)
    cost_R = np.diag(R)
    ocp.cost.W = scipy.linalg.block_diag(cost_Q, cost_R)
    ocp.cost.W_e = cost_Q

    ocp.cost.Vx = np.zeros((ny, nx))   # state
    ocp.cost.Vx[:nx,:nx] = np.eye(nx)

    ocp.cost.Vu = np.zeros((ny, nu))   # controls
    ocp.cost.Vu[-4:,-4:] = np.eye(nu)

    ocp.cost.Vx_e = np.eye(nx)         # terminal state

    ocp.cost.yref  = np.zeros((ny, ))
    ocp.cost.yref_e = np.zeros((ny_e, ))

    # set constraints
    ocp.constraints.x0 = x0  # initial constraints

    ocp.constraints.lbu = np.array([0.1*max_thrust, -max_omega_xy, -max_omega_xy, -max_omega_z])
    ocp.constraints.ubu = np.array([0.9*max_thrust, max_omega_xy, max_omega_xy, max_omega_z])
    ocp.constraints.idxbu = np.array([0, 1, 2, 3]) # control constraints
    
    ocp.constraints.lbx = np.array([-max_v, -max_v, -max_v])
    ocp.constraints.ubx = np.array([max_v, max_v, max_v])
    ocp.constraints.idxbx = np.array([7, 8, 9]) # velocity constraints

    # set QP solver and integration
    ocp.solver_options.tf = Tf
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 3
    ocp.solver_options.nlp_solver_max_iter = 200
    ocp.solver_options.tol = 1e-4

    # set acados export directory & model file
    ocp.code_export_directory = os.path.join(get_package_share_directory('ls2n_mpc_single_drone'), 'acados_code')
    model_file = os.path.join(
        os.path.relpath(get_package_share_directory('ls2n_mpc_single_drone'), os.curdir),
        model.name + "_" + "acados_ocp.json"
    )
    # create solver
    acados_solver = AcadosOcpSolver(ocp, json_file=model_file)

    return acados_solver
