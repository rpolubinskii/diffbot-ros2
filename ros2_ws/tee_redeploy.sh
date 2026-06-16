git pull
colcon build
source install/setup.sh

export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{time}] [{name}]: {message}"
ros2 launch diffbot diffbot.launch.py 2>&1 | tee ~/diffbot_$(date +%F_%H%M%S).log