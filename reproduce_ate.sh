#!/bin/bash
set -e

# ================= Configuration =================
CONTAINER_NAME="vins_container"
IMAGE_NAME="ros:vins-mono"
HOST_WS="/home/cheng/catkin_ws"
CONTAINER_WS="/root/catkin_ws"
HOST_DATA="/home/cheng/nya_01"
CONTAINER_DATA="/data"
BAG_NAME="nya_01.bag"
HOST_OUTPUT="$HOST_WS/vins_output"
# =================================================

echo "=============================================="
echo "   VINS-Mono Benchmarking (Docker Edition)    "
echo "=============================================="

# 1. Check/Start Container
echo "[Infra] Checking Docker Container: $CONTAINER_NAME..."
if [ ! "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
    if [ "$(docker ps -aq -f status=exited -f name=$CONTAINER_NAME)" ]; then
        echo "        Container exists but stopped. Starting..."
        docker start $CONTAINER_NAME
    else
        echo "        Container not found. Creating and starting..."
        # Note: Added --net=host and privileged for hardware access if needed, though mostly using bag here.
        # Added volume mounts reflecting PROJECT_CONTEXT.md
        docker run -d --name $CONTAINER_NAME \
            --net=host \
            --privileged \
            --entrypoint /bin/bash \
            -v $HOST_WS:$CONTAINER_WS \
            -v $HOST_DATA:$CONTAINER_DATA \
            -e DISPLAY=$DISPLAY \
            -v /tmp/.X11-unix:/tmp/.X11-unix \
            $IMAGE_NAME -c "tail -f /dev/null"
    fi
else
    echo "        Container is already running."
fi

# 2. Setup VINS Environment inside Docker
echo "[Setup] Cleaning previous state..."
# Force kill potential zombies from previous runs
# Force kill potential zombies from previous runs
docker exec $CONTAINER_NAME bash -c "pkill -f ros[m]aster; pkill -f ros[o]ut; pkill -f ros[b]ag; pkill play; pkill vins_estimator; pkill feature_tracker; pkill benchmark_publisher || true"
# Remove old outputs to prevent stale data reading
docker exec $CONTAINER_NAME bash -c "rm -f /root/catkin_ws/vins_output/*.csv"

echo "[Setup] Sourcing workspace in container..."
# Ensure output directory exists and copy Ground Truth
docker exec $CONTAINER_NAME bash -c "mkdir -p /root/catkin_ws/vins_output"
docker exec $CONTAINER_NAME bash -c "cp /root/catkin_ws/src/VINS-Mono/benchmark_publisher/config/nya_01/data.csv /root/catkin_ws/vins_output/leica_pose.csv"

# BUILD WORKSPACE
echo "[Build] Building workspace inside Docker..."
# Build with specific ROS version (Kinetic identified)
docker exec $CONTAINER_NAME bash -c "source /opt/ros/kinetic/setup.bash; catkin_make -j2"


# 3. Launch VINS (Detached background process inside Docker?) 
# Running detached ROS commands via docker exec is tricky. 
# We will run it in background with nohup or just & inside the bash -c.
echo "[Run] Launching VINS-Mono inside Docker (Synchronous)..."
echo "      (This will block until bag playback finishes ~2 min)"

# We run roslaunch in foreground. 'autorun:=true' makes rosbag play a required node, 
# so roslaunch will exit when bag finishes.
# We also export USER=root to fix the launch file error.
docker exec $CONTAINER_NAME bash -c "export USER=root; source /opt/ros/kinetic/setup.bash; source $CONTAINER_WS/devel/setup.bash; roslaunch vins_estimator run_ntuviral.launch bag_file:=$CONTAINER_DATA/$BAG_NAME autorun:=true enable_rviz:=false log_dir:=$CONTAINER_WS/vins_output"

echo "[Info] Bag playback finished (Launch exited)."

# 4. cleanup
# 4. cleanup (Double check)
echo "[Cleanup] Stopping container processes..."
docker exec $CONTAINER_NAME bash -c "source /opt/ros/kinetic/setup.bash; rosnode kill -a" || true
docker exec $CONTAINER_NAME bash -c "pkill -f ros[m]aster; pkill -f ros[b]ag; pkill play; pkill vins_estimator; pkill feature_tracker; pkill benchmark_publisher || true"

# 5. Evaluate on HOST
# The output file is in $HOST_OUTPUT (mapped from container)
echo "[Analysis] Running Evaluation on HOST..."
# Check if python3 and scipy/numpy are available on host (User said host is linux, likely has them)
if python3 -c "import numpy; import scipy" &> /dev/null; then
    python3 "$HOST_WS/evaluate_viral.py" "$HOST_OUTPUT/leica_pose.csv" "$HOST_OUTPUT/vins_result_loop.csv"
else
    echo "[Warn] Python dependencies missing on Host. Trying inside Docker..."
    docker exec $CONTAINER_NAME bash -c "python $CONTAINER_WS/evaluate_viral.py /root/catkin_ws/vins_output/leica_pose.csv /root/catkin_ws/vins_output/vins_result_loop.csv"
fi
