#!/usr/bin/env python3
import sqlite3
import sys
import pandas as pd
import matplotlib.pyplot as plt
import os

# -----------------------------
# CONFIG: set bag file path here or pass as argument
# Example:
# python3 plot_odom.py path/to/session_XXXX.db3
# -----------------------------
bag_path = sys.argv[1] if len(sys.argv) > 1 else input("Enter path to .db3 bag file: ")

if not os.path.isfile(bag_path):
    print(f"Bag file not found: {bag_path}")
    sys.exit(1)

# Connect to SQLite bag
conn = sqlite3.connect(bag_path)
cursor = conn.cursor()

# Topics to extract
odom_topics = ["/odom_est", "/odom_lidar_fused"]

# Helper: convert blob to ROS Odometry message
def extract_pose(blob):
    import rclpy.serialization as ser
    from nav_msgs.msg import Odometry
    msg = ser.deserialize_message(blob, Odometry)
    return msg.pose.pose.position.x, msg.pose.pose.position.y

dfs = {}

# Extract odometry topics
for topic in odom_topics:
    cursor.execute("SELECT id FROM topics WHERE name=?", (topic,))
    row = cursor.fetchone()
    if not row:
        print(f"Topic {topic} not found in bag!")
        continue
    topic_id = row[0]

    cursor.execute("SELECT data FROM messages WHERE topic_id=?", (topic_id,))
    xs, ys = [], []
    for (data,) in cursor.fetchall():
        x, y = extract_pose(data)
        xs.append(x)
        ys.append(y)
    dfs[topic] = pd.DataFrame({"x": xs, "y": ys})

conn.close()

# Plot all odometry trajectories
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
