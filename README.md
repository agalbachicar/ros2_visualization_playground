# ros2_visualization_playground

## Guidelines

Base project docker image with ros2 foxy full distribution and a few sample
packages to show off the power of RQT and PlotJuggler.

## Docker

- NVIDIA GPU support - *Skip this step if you don't have an NVIDIA graphics card*

Make sure you have the drivers installed:

```sh
nvidia-smi
```

Install `nvidia-container-toolkit` in your host machine:

```sh
sudo apt-get install -y nvidia-container-toolkit
```

- Build the docker image whose name is `ros2_foxy`:

```sh
./docker/build.sh
```

You can also try to set a specific image name:

```sh
./docker/build.sh -i my_fancy_image_name
```

- Run a docker container from `ros2_foxy` called `ros2_foxy_container`:

```sh
./docker/run.sh
```

You can also try to set specific image and container names:

```sh
./docker/run.sh -i my_fancy_image_name -c my_fancy_container_name
```

And a prompt in the docker image should appear at the root of the workspace:

```sh
$ pwd
/home/username/ws
$ ls -lash src/
total 12K
4.0K drwxr-xr-x 3 root         root         4.0K Nov 25 20:53 .
4.0K drwxr-xr-x 1 root         root         4.0K Nov 25 20:53 ..
4.0K drwxrwxr-x 4 agalbachicar agalbachicar 4.0K Nov 25 19:20 ros2_visualization_playground
```

## Prepare your workspace, build and test

- To install dependencies via `rosdep`:

```sh
rosdep install -i -y --rosdistro foxy --from-paths src
```
- To build:

```sh
colcon build
```

And if you want details of the run commands:

```sh
colcon build --event-handlers console_direct+
```

- To test:

```sh
colcon test --event-handlers console_direct+
colcon test-result
```

## Try the code!

After building, source your install space:

```sh
source install/setup.bash
```

Run a node to generate synthetic telemetry data:

```sh
ros2 run telemetry_data_generation telemetry_data_node
```

Run the quaternion to euler angles conversion

```sh
ros2 run telemetry_visualization quat2euler_node
```

Finally, run the `rqt_gui` visualization:

```sh
ros2 run rqt_gui rqt_gui --perspective-file install/telemetry_visualization/config/perspectives/pose.perspective
```