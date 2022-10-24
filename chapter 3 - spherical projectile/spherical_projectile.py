import sys
import os

sys.path.append(os.path.split(sys.path[0])[0])
import matplotlib.pyplot as plt
import numpy as np
import simulation_parameters as SIM

import aerosonde_parameters as MAV
from rotations import Euler2Quaternion
from rotations import Euler2Rotation
from rotations import Rotation2Quaternion
from rotations import Quaternion2Euler

from mav_viewer import MavViewer
from data_viewer import DataViewer
from mav_dynamics3 import MavDynamics
from msg_delta import MsgDelta

# initialize the visualization
VIDEO = False  # True==write video, False==don't write video
ANIMATE = True
if ANIMATE:
    mav_view = MavViewer()  # initialize the mav viewer
    data_view = DataViewer()  # initialize view of data plots
if VIDEO and ANIMATE:
    from video_writer import VideoWriter
    video = VideoWriter(video_name="projectile_test.mp4",
                        bounding_box=(0, 0, 1000, 1000),
                        output_rate=SIM.ts_video)

# initialize elements of the architecture
mav = MavDynamics(SIM.ts_simulation)
delta = MsgDelta()

# initialize the simulation time
sim_time = SIM.start_time

# define initial conditions for projectile
north0 = 0.
east0 = 0.
altitude0 = 0.001
u0 = 30
phi0 = np.pi * (0)  # initial roll angle
theta0 = np.pi * (1/4)  # initial pitch angle
psi0 = np.pi * (0)  # initial yaw angle
e = Euler2Quaternion(phi0, theta0, psi0)
e0 = e.item(0)
e1 = e.item(1)
e2 = e.item(2)
e3 = e.item(3)
p0 = 2.5
q0 = 0
r0 = 0.
mav._state = np.array([
    [north0],         # (north0)
    [east0],         # (east0)
    [-altitude0],         # (down0)
    [u0],    # (u0)
    [0],    # (v0)
    [0.],    # (w0)
    [e0],    # (e0)
    [e1],    # (e1)
    [e2],    # (e2)
    [e3],    # (e3)
    [p0],    # (p0)
    [q0],    # (q0)
    [r0],    # (r0)
])
mav._update_true_state()

V0_z = u0 * np.sin(theta0)
V0_x = np.cos(psi0) * u0 * np.cos(theta0)
V0_y = np.sin(psi0) * u0 * np.cos(theta0)

t_flight_expected = (V0_z + np.sqrt(V0_z**2 + 2*MAV.gravity*altitude0))/MAV.gravity
n_land = north0 + t_flight_expected * V0_x
e_land = east0 + t_flight_expected * V0_y
if V0_z > 0:
    max_altitude = altitude0 + V0_z**2 / (2 * MAV.gravity)
    V_impact_z = np.sqrt(2*MAV.gravity*max_altitude)
else:
    max_altitude = altitude0
    V_impact_z = V0_z + np.sqrt(2*MAV.gravity*max_altitude)

V_impact = np.sqrt(V_impact_z**2 + V0_y**2 + V0_x**2)

last_max_alt = altitude0
rot_counter = 0
sim_time = 0
assert altitude0 >= 0
tArr = []
uArr = []
vArr = []
wArr = []
northArr = []
eastArr = []
downArr = []
e0Arr = []
e1Arr = []
e2Arr = []
e3Arr = []
pArr = []
qArr = []
rArr = []
phiArr = []
thetaArr = []
psiArr = []

while sim_time < t_flight_expected + 100:

    # this keeps gravity in z-direction of inertial frame
    Rb_i = Euler2Rotation(mav.true_state.phi, mav.true_state.theta, mav.true_state.psi).T
    f_i = np.array([0, 0, MAV.gravity*MAV.mass]).T
    f_b = Rb_i @ f_i
    forces_moments = np.array([[f_b[0], f_b[1], f_b[2], 0, 0, 0]]).T

    last_theta = mav.true_state.theta
    mav.update(forces_moments)  # propagate the MAV dynamics

    if ANIMATE:
        data_view.update(mav.true_state,  # true states
                        mav.true_state,  # estimated states
                        mav.true_state,  # commanded states
                        delta,  # inputs to the aircraft
                        SIM.ts_simulation)


        mav_view.update(mav.true_state)  # plot body of MAV

    if VIDEO and ANIMATE:
        video.update(sim_time)          # record frame

    if last_max_alt < mav.true_state.altitude:
        last_max_alt = mav.true_state.altitude

    tArr.append(sim_time)
    northArr.append(mav._state.item(0))
    eastArr.append(mav._state.item(1))
    downArr.append(mav._state.item(2))
    uArr.append(mav._state.item(3))
    vArr.append(mav._state.item(4))
    wArr.append(mav._state.item(5))
    e0Arr.append(mav._state.item(6))
    e1Arr.append(mav._state.item(7))
    e2Arr.append(mav._state.item(8))
    e3Arr.append(mav._state.item(9))
    pArr.append(mav._state.item(10))
    qArr.append(mav._state.item(11))
    rArr.append(mav._state.item(12))
    quaternion = np.array([mav._state.item(6), mav._state.item(7), mav._state.item(8), mav._state.item(9)])
    angles = Quaternion2Euler(quaternion)
    phiArr.append(angles[0])
    thetaArr.append(angles[1])
    psiArr.append(angles[2]/100)


    sim_time += SIM.ts_simulation

    rot_counter = rot_counter + np.abs(last_theta - mav.true_state.theta)

    # LANDING + COMPARE SIM AND PHYSICS VALUES
    if mav.true_state.altitude <= 0:
        ## FLIGHT TIME UNIT TEST
        if (sim_time - t_flight_expected) < SIM.ts_simulation:
            print("\nUnit test passed")
        else:
            print("Unit test failed")
        print("Expect to fall for %f and it was %f \n" %(t_flight_expected, sim_time))

        ## LANDING POSITION (NORTH) UNIT TEST
        n_pos_error = SIM.ts_simulation * max(np.abs(V0_x), 1)
        if np.abs(mav.true_state.north - n_land) < n_pos_error:
            print("Unit test passed")
        else:
            print("Unit test failed")
        print("Expect to land at %f N and it was %f N \n" %(n_land, mav.true_state.north))

        ## MAX ALTITUDE UNIT TEST
        alt_error = SIM.ts_simulation * max(np.abs(V0_z), 1)
        if np.abs(max_altitude - last_max_alt) < alt_error:
            print("Unit test passed")
        else:
            print("Unit test failed")
        print("Expected max height to be %f and it was %f \n" %(max_altitude, last_max_alt))

        # LANDING ORIENTATION
        print("Expect body to spin %f and it spun %f" %(q0*sim_time, rot_counter))

        break

if ANIMATE:
    input("Press any key to terminate the program")
    if VIDEO:
        video.close()

"""
plt.figure(0)
plt.plot(tArr, northArr, 'r')
plt.plot(tArr, eastArr, 'b')
plt.plot(tArr, downArr, 'g')
plt.grid()
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.legend(['North', 'East', 'Down'])
plt.title('Position vs time')

plt.figure(1)
plt.plot(tArr, uArr, 'r')
plt.plot(tArr, vArr, 'b')
plt.plot(tArr, wArr, 'g')
plt.grid()
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.legend(['Along x-axis', 'Along y-axis', 'Along z-axis'])
plt.title('Velocity vs time')
plt.show()

plt.figure(2)
plt.plot(tArr, phiArr, 'r')
plt.plot(tArr, thetaArr, 'b')
plt.plot(tArr, psiArr, 'g')
plt.grid()
plt.xlabel('Time (s)')
plt.ylabel('Angle (Radians)')
plt.legend(['Phi', 'Theta', 'Psi'])

plt.show()
"""