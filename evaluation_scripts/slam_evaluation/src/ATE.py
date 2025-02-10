import numpy as np
import pandas as pd
import os
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

# Function to align trajectories using Umeyama's method
def umeyama_alignment(source, target):
    """
    Aligns source trajectory to target trajectory using Umeyama's method.
    Returns the aligned source trajectory.
    """
    # Center the trajectories
    source_centered = source - np.mean(source, axis=0)
    target_centered = target - np.mean(target, axis=0)

    # Compute covariance matrix
    H = source_centered.T @ target_centered

    # Singular Value Decomposition (SVD)
    U, S, Vt = np.linalg.svd(H)

    # Compute rotation matrix
    R_mat = Vt.T @ U.T

    # Handle reflection case
    if np.linalg.det(R_mat) < 0:
        Vt[-1, :] *= -1
        R_mat = Vt.T @ U.T

    # Compute translation
    t = np.mean(target, axis=0) - np.mean(source, axis=0) @ R_mat.T

    # Apply transformation to source
    aligned_source = source @ R_mat.T + t

    return aligned_source

run_number = 4
for i in range(run_number):
    # Load data
    data_dir = os.getcwd() + f"/data/run{run_number}/"
    csv_name_x = "plot_data_x.csv"  # SLAM estimated pose
    csv_name_y = "plot_data_y.csv"  # Ground truth pose

    x_values = pd.read_csv(data_dir + csv_name_x)
    y_values = pd.read_csv(data_dir + csv_name_y)

    # Filter data
    gt_x = x_values[x_values["topic"] == "/waffle/pose.pose.position.x"].drop(columns=["topic"]).rename(columns={"value": "x"})
    gt_y = y_values[y_values["topic"] == "/waffle/pose.pose.position.y"].drop(columns=["topic"]).rename(columns={"value": "y"})

    slam_x = x_values[x_values["topic"] == "/pose.pose.pose.position.x"].drop(columns=["topic"]).rename(columns={"value": "x"})
    slam_y = y_values[y_values["topic"] == "/pose.pose.pose.position.y"].drop(columns=["topic"]).rename(columns={"value": "y"})

    # Merge data
    gt_df = pd.merge(gt_x, gt_y, on="timestamp", how="inner")
    slam_df = pd.merge(slam_x, slam_y, on="timestamp", how="inner")

    # Sort dataframes by timestamp
    gt_df = gt_df.sort_values(by="timestamp")
    slam_df = slam_df.sort_values(by="timestamp")

    # Merge dataframes based on nearest timestamps
    merged_df = pd.merge_asof(gt_df, slam_df, on="timestamp", direction="nearest")
    merged_df = merged_df.rename(columns={"x_x": "gt_x", "y_x": "gt_y", "x_y": "slam_x", "y_y": "slam_y"})

    # Extract ground truth and SLAM trajectories
    gt_trajectory = merged_df[["gt_x", "gt_y"]].to_numpy()
    slam_trajectory = merged_df[["slam_x", "slam_y"]].to_numpy()

    # Align SLAM trajectory to ground truth
    aligned_slam_trajectory = umeyama_alignment(slam_trajectory, gt_trajectory)

    # Compute Euclidean distances (ATE)
    errors = np.linalg.norm(gt_trajectory - aligned_slam_trajectory, axis=1)

    # Compute RMSE of ATE
    ate = np.sqrt(np.mean(errors**2))
    print(f"Absolute Trajectory Error (ATE): {ate:.4f} meters")

    # Plot trajectories
    plt.plot(gt_trajectory[:, 0], gt_trajectory[:, 1], label="Ground Truth", linestyle="solid", color="blue")
    plt.plot(aligned_slam_trajectory[:, 0], aligned_slam_trajectory[:, 1], label="Aligned SLAM", linestyle="dashed", color="red")
    plt.legend()
    plt.xlabel("X position (m)")
    plt.ylabel("Y position (m)")
    plt.title("SLAM vs Ground Truth Trajectories (ATE)")
    plt.suptitle("Arthur Alexandersson")
    plt.show()