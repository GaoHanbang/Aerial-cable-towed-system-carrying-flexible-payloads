from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
import os
import glob
from enum import IntEnum
import numpy as np
from scipy.interpolate import KroghInterpolator
from scipy.interpolate import CubicSpline
import pickle
import sys
import matplotlib.pyplot as plt


class InterpolationType(IntEnum):
    NO_INTERPOLATION = 0
    POLY = 1
    POLY5 = 2  # 5 order polynomial
    POLY7 = 3  # 7 order polynomial
    SPLINE = 4  # cubic spline


def generate_trajectories():
    # List all the trajectory files
    try:
        traj_directory = os.path.join(get_package_share_directory('ls2n_tools'),
                                      'trajectories')
    except PackageNotFoundError:
        exit(0)
    files = glob.glob(os.path.join(traj_directory, '*.traj'))
    for filename in files:
        # Open the file
        f = open(filename, "r")
        # Read the configuration lines
        line = f.readline()
        if line[0:12] != "description:":
            exit(filename + ": trajectory file should start with description")
        line = f.readline()
        if line[0:14] != "interpolation:":
            exit(filename + ": second line should be interpolation")
        interpolation_str = line[14:]
        interpolation_str = interpolation_str.strip()
        if interpolation_str == "none":
            interpolation = InterpolationType.NO_INTERPOLATION
        elif interpolation_str == "polynomial":
            interpolation = InterpolationType.POLY
        elif interpolation_str == "polynomial-5":
            interpolation = InterpolationType.POLY5
        elif interpolation_str == "polynomial-7":
            interpolation = InterpolationType.POLY7
        elif interpolation_str == "spline":
            interpolation = InterpolationType.SPLINE
        else:
            exit(filename + ": invalid interpolation type")
        line = f.readline()
        if line[0:9] != "sampling:":
            exit(filename + ": third line should be sampling time")
        sampling_str = line[9:]
        sampling_str = sampling_str.strip()
        try:
            sampling = float(sampling_str)
        except ValueError:
            exit(filename + ": invalid sampling type (should be a float)")
        if sampling < 0.001 and interpolation != InterpolationType.NO_INTERPOLATION:
            exit(filename + ": sampling cannot be lower than 0.001")
        # Get the variables
        line = f.readline()
        trajectory_in = {}
        variable_keys = line.split()
        n_var = len(variable_keys)
        if variable_keys[0] != "time":
            exit(filename + ": time should be first column in trajectory definition")
        for key in variable_keys:
            trajectory_in[key] = []

        # Extract the data
        last_time = -1.0
        n_line = 4
        for line in f:
            if line == "":
                break
            n_line += 1
            try:
                data = [float(x) for x in line.split()]
            except ValueError:
                exit(filename + ": non float value in line " + str(n_line))
            if len(data) != n_var:
                exit(filename + ": not matching variables count in line " + str(n_line))
            if data[0] <= last_time:
                exit(filename + ": time index should be strictly increasing in line " + str(n_line))
            for i, x in enumerate(data):
                trajectory_in[variable_keys[i]].append(x)
        if trajectory_in["time"][0] != 0.0:
            exit(filename + ": trajectory starting time should be 0.0")

        # Interpolation
        if interpolation == InterpolationType.NO_INTERPOLATION:
            trajectory_out = trajectory_in
        else:
            final_time = trajectory_in["time"][-1]
            trajectory_out = {"time": np.arange(0.0, final_time + sampling / 2, sampling)}  # adding final timestamp
            # Array to be able to set initial and final vel and acc to 0
            for variable in variable_keys:
                if variable != "time" and variable[-1] != "D":
                    if interpolation == InterpolationType.POLY:
                        time_for_interp = np.concatenate(([0.0, 0.0], trajectory_in["time"], [final_time, final_time]))
                        first_point = trajectory_in[variable][0]
                        last_point = trajectory_in[variable][-1]
                        y_for_interp = np.concatenate(([first_point, 0.0, 0.0],
                                                       trajectory_in[variable][1:-1],
                                                       [last_point, 0.0, 0.0]))
                        interpolated = KroghInterpolator(time_for_interp, y_for_interp)
                        trajectory_out[variable] = interpolated(trajectory_out["time"])
                        trajectory_out[variable + "D"] = interpolated.derivative(trajectory_out["time"], der=1)
                        trajectory_out[variable + "DD"] = interpolated.derivative(trajectory_out["time"], der=2)
                        trajectory_out[variable + "DDD"] = interpolated.derivative(trajectory_out["time"], der=3)

                    elif interpolation == InterpolationType.SPLINE:
                        interpolated = CubicSpline(trajectory_in["time"], trajectory_in[variable], bc_type="clamped")
                        trajectory_out[variable] = interpolated(trajectory_out["time"])
                        trajectory_out[variable + "D"] = interpolated.derivative(1)(trajectory_out["time"])
                        trajectory_out[variable + "DD"] = interpolated.derivative(2)(trajectory_out["time"])
                        trajectory_out[variable + "DDD"] = interpolated.derivative(3)(trajectory_out["time"])

                    else:
                        n_segment = n_line - 5  # number of trajectory segment (=n_waypoint-1)
                        for i in range(n_segment):
                            # Set points for the segment
                            t_start = trajectory_in["time"][i]
                            t_next = trajectory_in["time"][i + 1]
                            start_point = trajectory_in[variable][i]
                            next_point = trajectory_in[variable][i + 1]
                            if variable + "D" in variable_keys:
                                start_point_D = trajectory_in[variable+"D"][i]
                                next_point_D = trajectory_in[variable+"D"][i + 1]
                            else:
                                start_point_D = 0.0
                                next_point_D = 0.0
                            if variable + "DD" in variable_keys:
                                start_point_DD = trajectory_in[variable+"DD"][i]
                                next_point_DD = trajectory_in[variable+"DD"][i + 1]
                            else:
                                start_point_DD = 0.0
                                next_point_DD = 0.0
                            if variable + "DDD" in variable_keys:
                                start_point_DDD = trajectory_in[variable+"DDD"][i]
                                next_point_DDD = trajectory_in[variable+"DDD"][i + 1]
                            else:
                                start_point_DDD = 0.0
                                next_point_DDD = 0.0
                            # Interpolation
                            if interpolation == InterpolationType.POLY5:
                                time_for_interp = [t_start, t_start, t_start, t_next, t_next, t_next]
                                y_for_interp = np.concatenate(([start_point, start_point_D, start_point_DD],
                                                               [next_point, next_point_D, next_point_DD]))
                                interpolated = KroghInterpolator(time_for_interp, y_for_interp)
                                timestamp = np.arange(t_start, t_next + sampling / 2, sampling)
                                trajectory_segment = interpolated(timestamp)
                                trajectory_segmentD = interpolated.derivative(timestamp, der=1)
                                trajectory_segmentDD = interpolated.derivative(timestamp, der=2)
                                trajectory_segmentDDD = interpolated.derivative(timestamp, der=3)
                                if i == 0:
                                    trajectory_out[variable] = trajectory_segment
                                    trajectory_out[variable + "D"] = trajectory_segmentD
                                    trajectory_out[variable + "DD"] = trajectory_segmentDD
                                    trajectory_out[variable + "DDD"] = trajectory_segmentDDD
                                else:
                                    trajectory_out[variable] = np.concatenate((trajectory_out[variable],
                                                                               trajectory_segment[1:]))
                                    trajectory_out[variable + "D"] = np.concatenate((trajectory_out[variable + "D"],
                                                                                     trajectory_segmentD[1:]))
                                    trajectory_out[variable + "DD"] = np.concatenate((trajectory_out[variable + "DD"],
                                                                                      trajectory_segmentDD[1:]))
                                    trajectory_out[variable + "DDD"] = np.concatenate((trajectory_out[variable + "DDD"],
                                                                                       trajectory_segmentDDD[1:]))

                            if interpolation == InterpolationType.POLY7:
                                time_for_interp = [t_start, t_start, t_start, t_start, t_next, t_next, t_next, t_next]
                                y_for_interp = np.concatenate((
                                    [start_point, start_point_D, start_point_DD, start_point_DDD],
                                    [next_point, next_point_D, next_point_DD, next_point_DDD]))
                                interpolated = KroghInterpolator(time_for_interp, y_for_interp)
                                timestamp = np.arange(t_start, t_next + sampling / 2, sampling)
                                trajectory_segment = interpolated(timestamp)
                                trajectory_segmentD = interpolated.derivative(timestamp, der=1)
                                trajectory_segmentDD = interpolated.derivative(timestamp, der=2)
                                trajectory_segmentDDD = interpolated.derivative(timestamp, der=3)
                                if i == 0:
                                    trajectory_out[variable] = trajectory_segment
                                    trajectory_out[variable + "D"] = trajectory_segmentD
                                    trajectory_out[variable + "DD"] = trajectory_segmentDD
                                    trajectory_out[variable + "DDD"] = trajectory_segmentDDD
                                else:
                                    trajectory_out[variable] = np.concatenate(
                                        (trajectory_out[variable], trajectory_segment[1:]))
                                    trajectory_out[variable + "D"] = np.concatenate(
                                        (trajectory_out[variable + "D"], trajectory_segmentD[1:]))
                                    trajectory_out[variable + "DD"] = np.concatenate(
                                        (trajectory_out[variable + "DD"], trajectory_segmentDD[1:]))
                                    trajectory_out[variable + "DDD"] = np.concatenate(
                                        (trajectory_out[variable + "DDD"], trajectory_segmentDDD[1:]))

        # Write the generated trajectory
        filename_out = filename + '.pickle'
        file_out = open(filename_out, "wb")
        pickle.dump(trajectory_out, file_out)
        file_out.close()


def list_trajectories():
    traj_directory = os.path.join(get_package_share_directory('ls2n_tools'),
                                  'trajectories')
    files = glob.glob(os.path.join(traj_directory, '*.pickle'))
    for filename in files:
        filename = filename.split('/')[-1]
        print(filename[:-12])


def plot_trajectory():
    if len(sys.argv) < 2:
        exit("Please provide a trajectory to plot)")
    else:
        filename = os.path.join(get_package_share_directory('ls2n_tools'),
                                'trajectories', sys.argv[1] + ".traj.pickle")
        try:
            file = open(filename, 'rb')
        except FileNotFoundError:
            exit("This trajectory does not exist.")
        trajectory = pickle.load(file)
        fig_pos = plt.figure("Position")
        fig_vel = plt.figure("Velocity")
        fig_acc = plt.figure("Acceleration")
        fig_jerk = plt.figure("Jerk")
        for variable in trajectory.keys():
            if variable != "time":
                if variable[-1] == "D":
                    if variable[-2:] == "DD":
                        if variable[-3:] == "DDD":
                            plt.figure("Jerk")
                            plt.plot(trajectory["time"], trajectory[variable], label=variable)
                        else:
                            plt.figure("Acceleration")
                            plt.plot(trajectory["time"], trajectory[variable], label=variable)
                    else:
                        plt.figure("Velocity")
                        plt.plot(trajectory["time"], trajectory[variable], label=variable)
                else:
                    plt.figure("Position")
                    plt.plot(trajectory["time"], trajectory[variable], label=variable)
        fig_pos.legend()
        fig_vel.legend()
        fig_acc.legend()
        fig_jerk.legend()
        plt.show()
