**Under review at *Annals of Biomedical Engineering*.**

# invasiveness_evaluation_skin

ROS2 package for reading 14 strain channels and one force channel from a skin phantom via an Arduino Mega 2560.

## Build

With ROS 2 sourced, run:

```bash
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws
git clone https://github.com/Michi-Tsubaki/invasiveness_evaluation_skin.git src/invasiveness_evaluation_skin
rosdep install --from-paths src/invasiveness_evaluation_skin --ignore-src -y
colcon build --packages-select invasiveness_evaluation_skin
source install/setup.bash
```

Source `~/colcon_ws/install/setup.bash` in each new terminal.

## Calibrate

Upload `arduino_sketch/IIPArduinoReadSerial/IIPArduinoReadSerial.ino` to the Mega using the Arduino IDE. The sketch expects an external analog reference. Use your board's serial port in place of `/dev/ttyACM0`.

```bash
ros2 run invasiveness_evaluation_skin iip_serial_publisher.py --ros-args -p port:=/dev/ttyACM0
```

In a second terminal, start calibration:

```bash
ros2 run invasiveness_evaluation_skin iip_calibration.py --ros-args -p yaml_path:="$HOME/iip_calib.yaml"
```

Keep the phantom unloaded while the node collects baseline samples, then press Enter. Press Enter again and move the phantom through its working range. Press Ctrl-C to save the calibration. Stop the serial reader before starting acquisition.

## Run

```bash
ros2 launch invasiveness_evaluation_skin IIP_bringup.launch.py port:=/dev/ttyACM0 calib_yaml:="$HOME/iip_calib.yaml"
```

Read calibrated strain on `/iip_strain` and raw force on `/phantom_force`. For low-pass filtering, use `IIP_bringup_butterworth.launch.py` with the same arguments and set `sample_rate_hz` to the input rate.

In another terminal, choose a view:

| View | Command |
| --- | --- |
| Strain heatmap | `ros2 launch invasiveness_evaluation_skin IIP_heatmap.launch.xml` |
| Pain-equivalent arrows | `ros2 launch invasiveness_evaluation_skin IIP_pain_arrows.launch.xml` |
| Pain-equivalent time series | `ros2 launch invasiveness_evaluation_skin IIP_pain_plot.launch.xml` |

Keep the phantom still for the first two seconds of heatmap input to establish the offset.
