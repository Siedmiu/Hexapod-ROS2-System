### IMU
To use IMU get on your ESP32 code from  `Project_Testy_src other files/IMU_2_test/src/main.cpp` it would send data from IMU to serial.

Message in serial is formed in the next way `imu: Time [ms],Accel X [m/s^2],Accel Y [m/s^2],Accel Z [m/s^2],Angle X [deg],Angle Y [deg],Angle Z [deg]`. Tag imu: at the beginning suggest that it is imu data.

Then run `colcon build` and `source install/setup.bash` and `ros2 run hex_gz serial_to_imu.py` which would convert IMU data from serial to ros `/imu` topic (make sure it u want to use that topic, if for any reason you;d like to use another, modify it in the code). To plot data from simulation or real robot u should run `ros2 run hex_gz imu_topic_data_plotter.py`


### Hex reaction to IMU
If u want to test how Hexapod reacts on IMU data u should run `ros2 run hex_gz move_body_based_on_imu.py`, then the code would get data about IMU and move platform without moving foots, and adjust platform position to get rpy on 0 0 0.


### Updates from hackathon 14.05
- added imu module

- started implementing interface to controll robot movements from app (currently )

- found problem - cannot colcon build files in `Project/ros2_ws/src/pajak/pajak `


- file in `Test/src/run_all.sh` - run from one console simulation and server terminal, u can add there more bash scripts, to open more different terminals from one place

`chmod +x run_all.sh `

` ./run_all.sh ESP_IP `

change ESP_IP to your ESP_IP!!!

