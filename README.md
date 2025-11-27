# Planning

## Dependencies

Make sure you pull, build, and install `ap1_msgs`. 
You'll also need `ros-jazzy-lanelet2`. So `sudo apt install ros-jazzy-lanelet2` if you haven't already.

## Compilaton

> Note: If you want clangd to provide good autocorrect do: `colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1` and `cp build/compile_commands.json .`

Build with: `colcon build`.

## Usage

Planning takes in map data and outputs two objects: 
1. TargetPath. A list of waypoints relative to the car.
2. SpeedProfile. A list of speeds that control should follow.

These are then ingested by control.