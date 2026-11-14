Set shell = CreateObject("WScript.Shell")
shell.Run "wsl.exe -d Ubuntu-22.04 -- bash -lc ""source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash; python3 '/mnt/d/UTP/00 Reseach Project/ROS_2/warehouse_runner/control_app.py' >> ~/ros2_ws/control-app-logs/app.log 2>&1""", 0, False
