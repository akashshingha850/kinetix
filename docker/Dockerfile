# kinetix image (architecture.md §11): ROS 2 Jazzy for the T2/T3 rig (rclpy, mavros_msgs, tf2) + the Python deps the
# session loop needs. The full uv venv (pycolmap, open3d, torch) joins in K4; this image runs the demo loop only.
# Build: docker compose -f docker/compose.yaml build
FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-mavros-msgs ros-jazzy-tf2-ros-py \
        python3-numpy python3-yaml python3-opencv python3-pytest git \
    && rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    PYTHONPATH=/kinetix/src \
    PYTHONUNBUFFERED=1 \
    ROS_LOG_DIR=/tmp/ros_log \
    HOME=/tmp
WORKDIR /kinetix
# ros_entrypoint.sh (from the base image) sources /opt/ros/jazzy before the command
CMD ["python3", "-m", "kinetix", "--help"]
