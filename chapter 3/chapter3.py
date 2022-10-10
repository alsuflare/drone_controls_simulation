import sys
sys.path.append('..')
import numpy as np
import simulation_parameters as SIM

from mav_viewer import MavViewer
from data_viewer import DataViewer
from mav_dynamics import MavDynamics
from msg_delta import MsgDelta
from PyQt6 import QtWidgets
from PyQt6.QtWidgets import QApplication

#Initialize the visualization
VIDEO = False  # True==write video, False==don't write video
mav_view = MavViewer()  # initialize the mav viewer
data_view = DataViewer()  # initialize view of data plots
if VIDEO is True:
    from video_writer import VideoWriter
    video = VideoWriter(video_name="chap3_video.avi",
                        bounding_box=(0, 0, 1000, 1000),
                        output_rate=SIM.ts_video)

#Initialize elements of the architecture
mav = MavDynamics(SIM.ts_simulation)
delta = MsgDelta()

#Initialize the simulation time
sim_time = SIM.start_time

#Main simulation loop
print("Press Command-Q to exit...")
while sim_time < SIM.end_time:
    # -------vary forces and moments to check dynamics-------------
    fx = 0
    fy = 0  # 0
    fz = 0  # 10
    Mx = 0  # 0.1
    My = 0  # 0.1
    Mz = 0  # 0.1
    forces_moments = np.array([[fx, fy, fz, Mx, My, Mz]]).T

    # -------physical system-------------
    mav.update(forces_moments)  # propagate the MAV dynamics

    # -------update viewer-------------
    mav_view.update(mav.true_state)  # plot body of MAV
    data_view.update(mav.true_state,  # true states
                     mav.true_state,  # estimated states
                     mav.true_state,  # commanded states
                     delta,  # inputs to the aircraft
                     SIM.ts_simulation)
    if VIDEO is True:
        video.update(sim_time)

    # -------increment time-------------
    sim_time += SIM.ts_simulation

if VIDEO is True:
    video.close()