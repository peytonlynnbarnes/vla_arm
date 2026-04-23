#!/usr/bin/env bash
# Full TERM 0 reset per run_commands.txt + explicit orphan kill + DDS flush.
# Use before every Gazebo launch to ensure a clean slate.
set +e

echo "[nuke] pkill ros2 launch / nodes"
pkill -f 'ros2 launch' 2>/dev/null
pkill -f 'move_group' 2>/dev/null
pkill -f 'controller_manager' 2>/dev/null
pkill -f 'robot_state_publisher' 2>/dev/null
pkill -f 'parameter_bridge' 2>/dev/null
pkill -f 'ros_gz_sim' 2>/dev/null
pkill -f 'gz sim' 2>/dev/null
pkill -f 'ruby.*gz' 2>/dev/null
pkill -f 'spawner' 2>/dev/null
sleep 1

echo "[nuke] SIGKILL survivors"
# explicit -9 for anything still holding a ROS-related process
for pat in 'gz sim server' 'gz sim gui' 'ruby.*gz' 'parameter_bridge' \
           'robot_state_publisher' 'controller_manager' 'move_group' \
           'rviz2' 'joint_state_broadcaster' 'ros2 launch'; do
  pids=$(pgrep -f "$pat" 2>/dev/null | tr '\n' ' ')
  if [ -n "$pids" ]; then
    echo "[nuke]   $pat -> $pids"
    kill -9 $pids 2>/dev/null
  fi
done
sleep 1

echo "[nuke] flushing DDS daemon"
ros2 daemon stop 2>/dev/null
sleep 2
ros2 daemon start 2>/dev/null

echo "[nuke] done. survivors:"
ps -ef | grep -E "gz sim|parameter_bridge|robot_state_publisher|ruby.*gz|controller_manager|move_group|ros2 launch" \
       | grep -v grep || echo "  (none)"
