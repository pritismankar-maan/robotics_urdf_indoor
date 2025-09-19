#!/usr/bin/env python3
import sqlite3
import sys
import pandas as pd
import matplotlib.pyplot as plt
import os

# -----------------------------
# CONFIG: set your bag file path here
# Example:
# bag_path = "install/my_fusion/share/my_fusion/bags/session_20250919_064519/session_20250919_064519_0.db3"
# -----------------------------
bag_path = sys.argv[1] if len(sys.argv) > 1 else input("Enter path to .db3 bag file: ")

if not os.path.isfile(bag_path):
    print(f"Bag file not found: {bag_path}")
    sys.exit(1)

# Connect to the SQLite bag
conn = sqlite3.connect(bag_path)
cursor = conn.cursor()

# Get topics from database
cursor.execute("SELECT name, type FROM topics")
topics_info = cursor.fetchall()
print("Topics in bag:")
for t in topics_info:
    print(t)

# Define the topics you want to extract
odom_topics = ["/odom_est", "/odom_lidar_est", "/odom_imu_est", "/odom_fused"]

# Helper: convert blob to ROS message
def extract_pose(blob):
    import rclpy.serialization as ser
    from nav_msgs.msg import Odometry
    msg = ser.deserialize_message(blob, Odometry)
    return msg.pose.pose.position.x, msg.pose.pose.position.y

# Helper: extract x, y from /tf messages (ground truth)
def extract_tf_ground_truth(blob):
    import rclpy.serialization as ser
    from tf2_msgs.msg import TFMessage
    msg = ser.deserialize_message(blob, TFMessage)
    xs, ys = [], []
    for t in msg.transforms:
        # Filter for odom -> base_link
        if t.header.frame_id == "odom" and t.child_frame_id == "base_link":
            xs.append(t.transform.translation.x)
            ys.append(t.transform.translation.y)
    return xs, ys

# Dictionary to store dataframes
dfs = {}

# Extract odometry topics
for topic in odom_topics:
    cursor.execute("SELECT id FROM topics WHERE name=?", (topic,))
    row = cursor.fetchone()
    if not row:
        print(f"Topic {topic} not found in bag!")
        continue
    topic_id = row[0]

    cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id=?", (topic_id,))
    rows = cursor.fetchall()
    xs, ys = [], []
    for ts, data in rows:
        x, y = extract_pose(data)
        xs.append(x)
        ys.append(y)
    dfs[topic] = pd.DataFrame({"x": xs, "y": ys})

# Extract /tf ground truth if available
cursor.execute("SELECT id FROM topics WHERE name='/tf'")
row = cursor.fetchone()
if row:
    topic_id = row[0]
    cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id=?", (topic_id,))
    xs, ys = [], []
    for ts, data in cursor.fetchall():
        x_list, y_list = extract_tf_ground_truth(data)
        xs.extend(x_list)
        ys.extend(y_list)
    if xs and ys:
        dfs['/tf_ground_truth'] = pd.DataFrame({"x": xs, "y": ys})

conn.close()

# Plot all odometry trajectories + ground truth
plt.figure(figsize=(8, 6))
for topic, df in dfs.items():
    plt.plot(df['x'], df['y'], label=topic)
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.title("Robot Trajectories")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.show()
