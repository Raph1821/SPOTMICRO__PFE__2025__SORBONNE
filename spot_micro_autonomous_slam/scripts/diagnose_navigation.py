#!/usr/bin/env python3
"""
Navigation Diagnosis Script

Checks all critical topics and nodes for autonomous navigation.
"""

import rospy
import subprocess
import sys

def check_topic(topic_name, topic_type=None):
    """Check if a topic exists and is being published"""
    try:
        result = subprocess.run(['rostopic', 'list'], capture_output=True, text=True, timeout=2)
        if topic_name in result.stdout:
            # Check if it's being published
            info_result = subprocess.run(['rostopic', 'info', topic_name], capture_output=True, text=True, timeout=2)
            if 'Publishers:' in info_result.stdout and 'None' not in info_result.stdout:
                print(f"✓ {topic_name} - EXISTS and PUBLISHING")
                if topic_type:
                    hz_result = subprocess.run(['rostopic', 'hz', topic_name], capture_output=True, text=True, timeout=3)
                    print(f"  Rate: {hz_result.stdout.split(chr(10))[0] if hz_result.stdout else 'N/A'}")
                return True
            else:
                print(f"✗ {topic_name} - EXISTS but NOT PUBLISHING")
                return False
        else:
            print(f"✗ {topic_name} - DOES NOT EXIST")
            return False
    except Exception as e:
        print(f"✗ {topic_name} - ERROR: {e}")
        return False

def check_node(node_name):
    """Check if a node is running"""
    try:
        result = subprocess.run(['rosnode', 'list'], capture_output=True, text=True, timeout=2)
        if node_name in result.stdout:
            print(f"✓ {node_name} - RUNNING")
            return True
        else:
            print(f"✗ {node_name} - NOT RUNNING")
            return False
    except Exception as e:
        print(f"✗ {node_name} - ERROR: {e}")
        return False

def check_tf(frame1, frame2):
    """Check if TF transform exists"""
    try:
        result = subprocess.run(['rosrun', 'tf2_ros', 'tf2_echo', frame1, frame2], 
                              capture_output=True, text=True, timeout=2)
        if 'Exception' not in result.stderr:
            print(f"✓ TF {frame1} -> {frame2} - EXISTS")
            return True
        else:
            print(f"✗ TF {frame1} -> {frame2} - DOES NOT EXIST")
            return False
    except:
        print(f"✗ TF {frame1} -> {frame2} - ERROR")
        return False

def main():
    rospy.init_node('diagnose_navigation', anonymous=True)
    
    print("=" * 60)
    print("NAVIGATION DIAGNOSIS")
    print("=" * 60)
    
    print("\n[1] Checking Critical Topics:")
    print("-" * 60)
    topics_ok = True
    topics_ok &= check_topic('/cmd_vel', 'geometry_msgs/Twist')
    topics_ok &= check_topic('/odom', 'nav_msgs/Odometry')
    topics_ok &= check_topic('/map', 'nav_msgs/OccupancyGrid')
    topics_ok &= check_topic('/scan', 'sensor_msgs/LaserScan')
    topics_ok &= check_topic('/move_base/goal', 'move_base_msgs/MoveBaseActionGoal')
    topics_ok &= check_topic('/move_base/status', 'actionlib_msgs/GoalStatusArray')
    
    print("\n[2] Checking Critical Nodes:")
    print("-" * 60)
    nodes_ok = True
    nodes_ok &= check_node('/odom_publisher')
    nodes_ok &= check_node('/move_base')
    nodes_ok &= check_node('/autonomous_explorer')
    nodes_ok &= check_node('/hector_mapping')
    nodes_ok &= check_node('/pybullet_sim')
    
    print("\n[3] Checking TF Transforms:")
    print("-" * 60)
    tf_ok = True
    tf_ok &= check_tf('odom', 'base_footprint')
    tf_ok &= check_tf('map', 'odom')
    tf_ok &= check_tf('map', 'base_footprint')
    
    print("\n[4] Checking move_base Status:")
    print("-" * 60)
    try:
        result = subprocess.run(['rostopic', 'echo', '-n', '1', '/move_base/status'], 
                              capture_output=True, text=True, timeout=3)
        if result.stdout:
            print("✓ move_base status available")
            print(result.stdout[:200])  # Print first 200 chars
        else:
            print("✗ move_base status not available")
    except:
        print("✗ Could not get move_base status")
    
    print("\n" + "=" * 60)
    if topics_ok and nodes_ok and tf_ok:
        print("SUMMARY: All checks passed!")
    else:
        print("SUMMARY: Some checks failed - see above for details")
    print("=" * 60)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

