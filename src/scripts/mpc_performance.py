#!/usr/bin/env python3
"""Analyze MPC tracking performance from a rosbag."""
import argparse
import numpy as np
import rosbag
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description="Evaluate MPC response")
    parser.add_argument("bag", help="path to rosbag with /next_base_path and /history_path")
    parser.add_argument("--output", default="mpc_performance.png", help="output figure path")
    args = parser.parse_args()

    ref_positions = []
    actual_positions = []
    timestamps = []

    bag = rosbag.Bag(args.bag)
    for topic, msg, t in bag.read_messages(topics=["/next_base_path", "/history_path"]):
        if topic == "/next_base_path":
            # assume first pose sequence is reference
            for pose in msg.poses:
                ref_positions.append([pose.position.x, pose.position.y, pose.position.z])
        elif topic == "/history_path":
            for pose in msg.poses:
                actual_positions.append([pose.position.x, pose.position.y, pose.position.z])
                timestamps.append(t.to_sec())
    bag.close()

    ref_positions = np.array(ref_positions)
    actual_positions = np.array(actual_positions)
    timestamps = np.array(timestamps)

    min_len = min(len(ref_positions), len(actual_positions))
    if min_len == 0:
        raise RuntimeError("Not enough data in bag")

    ref_positions = ref_positions[:min_len]
    actual_positions = actual_positions[:min_len]
    timestamps = timestamps[:min_len]
    timestamps -= timestamps[0]

    errors = np.linalg.norm(actual_positions - ref_positions, axis=1)
    mean_error = float(np.mean(errors))

    below_thresh = np.where(errors < 0.1)[0]
    response_time = float(timestamps[below_thresh[0]]) if len(below_thresh) else float("nan")

    plt.figure()
    plt.plot(timestamps, errors)
    plt.xlabel("Time [s]")
    plt.ylabel("Position error [m]")
    plt.title("MPC tracking error")
    plt.grid(True)
    plt.savefig(args.output)
    print(f"Response time: {response_time:.3f}s, mean error: {mean_error:.3f}m")


if __name__ == "__main__":
    main()
