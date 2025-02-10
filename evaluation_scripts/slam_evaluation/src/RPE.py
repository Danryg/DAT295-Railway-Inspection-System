import numpy as np
import pandas as pd
import os
import matplotlib.pyplot as plt

def quaternion_to_theta(qx, qy, qz, qw):
    theta = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy**2 + qz**2))
    return theta

def compute_relative_poses(trajectory, step_size):
    relative_poses = []
    for i in range(len(trajectory) - step_size):
        # Current and next pose
        pose_current = trajectory[i]
        pose_next = trajectory[i + step_size]

        # Compute relative translation and rotation
        delta_x = pose_next[0] - pose_current[0]
        delta_y = pose_next[1] - pose_current[1]
        delta_theta = pose_next[2] - pose_current[2]

        # Normalize angle to [-pi, pi]
        delta_theta = np.arctan2(np.sin(delta_theta), np.cos(delta_theta))

        relative_poses.append([delta_x, delta_y, delta_theta])
    return np.array(relative_poses)

# Function to compute RPE
def compute_rpe(gt_trajectory, slam_trajectory, step_size):
    # Compute relative poses
    gt_relative_poses = compute_relative_poses(gt_trajectory, step_size)
    slam_relative_poses = compute_relative_poses(slam_trajectory, step_size)

    # Compute translation and rotation errors
    translation_errors = np.linalg.norm(gt_relative_poses[:, :2] - slam_relative_poses[:, :2], axis=1)
    rotation_errors = np.abs(gt_relative_poses[:, 2] - slam_relative_poses[:, 2])

    # Compute RMSE for translation and rotation errors
    translation_rmse = np.sqrt(np.mean(translation_errors**2))
    rotation_rmse = np.sqrt(np.mean(rotation_errors**2))

    return translation_errors, rotation_errors, translation_rmse, rotation_rmse

# Main script
run_number = 4  # Number of runs
step_size = 10  # Step size for RPE computation
translation_errors_all = []  # Store translation errors for each run
rotation_errors_all = []  # Store rotation errors for each run
translation_rmse_values = []  # Store translation RMSE for each run
rotation_rmse_values = []  # Store rotation RMSE for each run

for i in range(1, run_number + 1):  # Loop through runs (1 to run_number)
    # Load data
    data_dir = os.getcwd() + f"/data/rpe/run{i}/"  # Use the correct run directory
    slam_data= "estimated_pose.csv"  # SLAM estimated pose
    gt_data = "ground_truth_pose.csv"  # Ground truth pose

    slam_df= pd.read_csv(data_dir + slam_data)
    gt_df = pd.read_csv(data_dir + gt_data)


    # Sort dataframes by timestamp
    gt_df = gt_df.sort_values(by="timestamp")
    slam_df = slam_df.sort_values(by="timestamp")

    # Merge dataframes based on nearest timestamps
    merged_df = pd.merge_asof(gt_df, slam_df, on="timestamp", direction="nearest")

    # Rename columns to include quaternion components
    merged_df = merged_df.rename(columns={
        "x_x": "gt_x", "y_x": "gt_y",  # Ground truth position
        "qx_x": "gt_qx", "qy_x": "gt_qy", "qz_x": "gt_qz", "qw_x": "gt_qw",  # Ground truth quaternion
        "x_y": "slam_x", "y_y": "slam_y",  # SLAM position
        "qx_y": "slam_qx", "qy_y": "slam_qy", "qz_y": "slam_qz", "qw_y": "slam_qw"  # SLAM quaternion
    })
    # Extract ground truth and SLAM trajectories (x, y, theta)
    gt_trajectory = merged_df[["gt_x", "gt_y", "gt_qx", "gt_qy", "gt_qz", "gt_qw"]].to_numpy()
    slam_trajectory = merged_df[["slam_x", "slam_y", "slam_qx", "slam_qy", "slam_qz", "slam_qw"]].to_numpy()


    # Convert quaternion to theta
    gt_trajectory[:, 2] = quaternion_to_theta(gt_trajectory[:, 2], gt_trajectory[:, 3], gt_trajectory[:, 4], gt_trajectory[:, 5])
    slam_trajectory[:, 2] = quaternion_to_theta(slam_trajectory[:, 2], slam_trajectory[:, 3], slam_trajectory[:, 4], slam_trajectory[:, 5])

    # Keep only x, y, theta
    gt_trajectory = gt_trajectory[:, :3]
    slam_trajectory = slam_trajectory[:, :3]

    # Compute RPE
    translation_errors, rotation_errors, translation_rmse, rotation_rmse = compute_rpe(gt_trajectory, slam_trajectory, step_size)
    print(f"Run {i} - Translation RMSE: {translation_rmse:.4f} meters")
    print(f"Run {i} - Rotation RMSE: {rotation_rmse:.4f} radians")

    # Store errors for plotting
    translation_errors_all.extend(translation_errors)
    rotation_errors_all.extend(rotation_errors)
    translation_rmse_values.append(translation_rmse)
    rotation_rmse_values.append(rotation_rmse)

# Plot translation errors over time
plt.figure()
plt.plot(translation_errors_all, label="Translation Error")
plt.xlabel("Pose Index")
plt.ylabel("Error (meters)")
plt.title("Translation Error Over Time")
plt.suptitle("Arthur Alexandersson")
plt.legend()
plt.show()

# Plot rotation errors over time
plt.figure()
plt.plot(rotation_errors_all, label="Rotation Error")
plt.xlabel("Pose Index")
plt.ylabel("Error (radians)")
plt.title("Rotation Error Over Time")
plt.suptitle("Arthur Alexandersson")
plt.legend()
plt.show()

# Plot histograms of errors
plt.figure()
plt.hist(translation_errors_all, bins=20, label="Translation Errors")
plt.xlabel("Error (meters)")
plt.ylabel("Frequency")
plt.title("Distribution of Translation Errors")
plt.suptitle("Arthur Alexandersson")
plt.legend()
plt.show()

plt.figure()
plt.hist(rotation_errors_all, bins=20, label="Rotation Errors")
plt.xlabel("Error (radians)")
plt.ylabel("Frequency")
plt.title("Distribution of Rotation Errors")
plt.suptitle("Arthur Alexandersson")
plt.legend()
plt.show()

# Plot RMSE values for each run
plt.figure()
x = np.arange(1, run_number + 1)
width = 0.35
plt.bar(x - width/2, translation_rmse_values, width, label="Translation RMSE")
plt.bar(x + width/2, rotation_rmse_values, width, label="Rotation RMSE")
plt.xlabel("Run Number")
plt.ylabel("RMSE")
plt.title("RMSE of Translation and Rotation Errors Across Runs")
plt.xticks(x)
plt.legend()
plt.show()