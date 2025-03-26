from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
import math as m
import sys
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import matplotlib.patches as patches

#sys.exit()
# Connect to robot
rtde_c = RTDEControl("192.168.1.125")
rtde_r = RTDEReceive("192.168.1.125")

# Parameters
dt = 1.0/500  # 2ms

init_pose = rtde_r.getActualTCPPose()
#print(rtde_r.getActualQ())
init_q = np.array([-0.07116967836488897, -2.2907940349974574, -1.6120822429656982, -1.0600456160357972, 0.8726012706756592, 0.6463595032691956])

joint_q = init_q
joint_speed = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
ratio = 0.8

# Move to initial joint position
# rtde_c.moveJ(joint_q,1.05*ratio,1.4*ratio)

# seconds = 15
# final_seconds = 10
# count_i = 0
# acceleration = 40

# q_ref_deg_j3 = 40
# q_ref_deg_j5 = 40
# final_q_ref = np.array([m.radians(q_ref_deg_j3), m.radians(q_ref_deg_j5)])

# A = m.radians(30)  # 30-degree amplitude
# f = 0.1  # Frequency in Hz

# q_cur = np.array([init_q[3], init_q[5]])

# k_p = 20

# # Lists to store values for plotting
# error_q_list = []
# q_ref_list = []
# q_cur_list = []
# obstacle_info_list = []
# cbf_data_list = []
# time_list = []

# #MF CBF
# temp_center_x = 0.38
# temp_center_y = 0.35 
# obstacle_center = np.array([temp_center_x, temp_center_y])
# distance_obs = 0
# obstacle_r = 0.1
# safety_margin = 1
# obstacle_info = np.array([temp_center_x, temp_center_y,obstacle_r])
# # Execute 500Hz control loop for 'seconds' seconds
# while count_i <= 500 * seconds:
#     t_start = rtde_c.initPeriod()
#     count_i += 1
#     time_elapsed = count_i * dt  # Time in seconds
#     curr_q = rtde_r.getActualQ()
#     q_cur[0] = curr_q[3]
#     q_cur[1] = curr_q[5]
    
#     q_ref = np.array([
#             (m.radians(q_ref_deg_j3)/final_seconds)*time_elapsed,
#             (m.radians(q_ref_deg_j5)/final_seconds)*time_elapsed,
#         ])
#      # Sine wave reference trajectory
#     if abs(q_ref[0]) >= m.radians(q_ref_deg_j3):
#         q_ref[0] = m.radians(q_ref_deg_j3)
#     if abs(q_ref[1]) >= m.radians(q_ref_deg_j5):
#         q_ref[1] = m.radians(q_ref_deg_j5)


#     # Compute error
#     error_q = q_ref - q_cur
#     nominal_ctrl_law = k_p * error_q

#     #MFCBF
 
#     d = np.array([0,0])
#     d = q_cur - obstacle_center
#     n_o_t = np.transpose(d)*(1/(np.linalg.norm(d))) 
#     distance_obs = np.linalg.norm(d)-obstacle_r*safety_margin

#     alpha_h = 0.8


#     temp_control_input = -n_o_t@nominal_ctrl_law-alpha_h*(distance_obs)

#     #print(n_o_t)
#     safety_vel = nominal_ctrl_law + max(temp_control_input,0)*np.transpose(n_o_t)

#     ctrl_law = safety_vel

#     # Save data for plotting
#     cbf_data = np.array([distance_obs, alpha_h])
#     error_q_list.append(error_q.copy())
#     q_cur_list.append(q_cur.copy())
#     q_ref_list.append(q_ref.copy())
#     time_list.append(count_i * dt)
#     obstacle_info_list.append(obstacle_info.copy())
#     cbf_data_list.append(cbf_data.copy())

#     # Safety limit check
#     if abs(curr_q[3]) >= m.pi/2 or abs(curr_q[5]) >= m.pi/2:
#         joint_speed[3] = 0
#         joint_speed[5] = 0
#         acceleration = 0
#         rtde_c.speedJ(joint_speed, acceleration, dt)
#         rtde_c.speedStop()
#         print("Limit reached")
#         break
#     else:
#         print("Control Running :: ", error_q)
#         joint_speed[3] = ctrl_law[0]
#         joint_speed[5] = ctrl_law[1]
#         rtde_c.speedJ(joint_speed, acceleration, dt)

#     rtde_c.waitPeriod(t_start)

rtde_c.speedStop()
rtde_c.stopScript()

# # Convert lists to numpy arrays for easy plotting
# error_q_array = np.array(error_q_list)
# q_cur_array = np.array(q_cur_list)
# q_ref_array = np.array(q_ref_list)
# obstacle_info_array = np.array(obstacle_info_list)
# cbf_data_array = np.array(cbf_data_list)
# # Create DataFrame
# df = pd.DataFrame({
#     "Time (s)": time_list,
#     "Error J3 (rad)": error_q_array[:, 0],
#     "Error J5 (rad)": error_q_array[:, 1],
#     "J3 Position (rad)": q_cur_array[:, 0],
#     "J5 Position (rad)": q_cur_array[:, 1],   
#     "J3 Reference (rad)": q_ref_array[:, 0],
#     "J5 Reference (rad)": q_ref_array[:, 1],
#     "center_x": obstacle_info_array[:, 0],
#     "center_y": obstacle_info_array[:, 1],
#     "radius": obstacle_info_array[:, 2],
#     "h": cbf_data_array[:, 0],
#     "alpha": cbf_data_array[:, 1],
# })

# # Save to CSV
# df.to_csv("joint_data.csv", index=False)
# print("Data saved to joint_data.csv")


# # Plot results
# plt.figure(figsize=(10, 5))
# # Plot error_q
# plt.subplot(3, 1, 1)
# plt.plot(time_list, error_q_array[:, 0], label="Error J3")
# plt.plot(time_list, error_q_array[:, 1], label="Error J5")
# plt.xlabel("Time (s)")
# plt.ylabel("Error (rad)")
# plt.legend()
# plt.title("Error in Joint Positions")

# # Plot q_cur
# plt.subplot(3, 1, 2)
# plt.plot(time_list, q_cur_array[:, 0], label="J3 Position")
# plt.plot(time_list, q_ref_array[:, 0], label="J3 Ref")

# plt.xlabel("Time (s)")
# plt.ylabel("Joint Positions (rad)")
# plt.legend()
# plt.title("Actual Joint Positions")

# plt.subplot(3, 1, 3)
# plt.plot(time_list, q_cur_array[:, 1], label="J5 Position")
# plt.plot(time_list, q_ref_array[:, 1], label="J5 Ref")

# plt.xlabel("Time (s)")
# plt.ylabel("Joint Positions (rad)")
# plt.legend()
# plt.title("Actual Joint Positions")

# plt.tight_layout()
# plt.show()

# plt.figure(figsize=(10, 5))
# # Plot error_q
# plt.plot(q_cur_array[:, 0], q_cur_array[:, 1], label="J3-J5")
# circle = patches.Circle((temp_center_x, temp_center_y), obstacle_r, color='r', fill=False, linestyle='dashed', linewidth=2, label="Unsafe Region")

# # Get current axis and add the circle
# ax = plt.gca()
# ax.add_patch(circle)
# plt.axis('equal')
# plt.xlabel("J3 (rad)")
# plt.ylabel("J5 (rad)")
# plt.legend()
# plt.title("Joint Positions")


# plt.tight_layout()
# plt.show()