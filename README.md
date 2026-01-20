## Universal Robots Setup (RobotRave Hackathon)

#### ROS Docker Image
ROS Setup Dockerfile can be found inside `./ur_ros_simulator_container`
Build:
```
cd ./ur_ros_simulator_container
docker build -t ur-ros-img .
```

Run with X11 for simulation GUI sharing with host system and shared host network to make setup server/client comms easier as well as volume mount for the `./src_ur_robot_server` from the host system:
```
sudo docker run -it --rm --network host --device /dev/dri:/dev/dri --env DISPLAY=$DISPLAY --env QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix -v <repo-root>/src_ur_robot_server:/home/<user>/workspace --name ur-container ur-ros-img
```

#### System Architecture
##### UR Robot Control Server
For robot controls we setup a server that interacts with the robot over the static-ip assigned over a direct ethernet connection to the UR Robot. The server's implementation can be found at `./src_ur_robot_controller`, it sets up a simple WebSocket interface that abstracts out all the core commands we needed such as relative/absolute joint movements, getting joint states and health checks. This server needs to be launched inside the running Docker container instance. To launch it exec into the docker instance and run the server, all the packages are pre-installed in the docker image already. Run:
```
sudo docker exec -it ur-container bash
cd /home/<user>/workspace/ # Mapped to the src folder of the server codebase on your host system
python main.py
```

#### UR Robot Control Client
The Robot Control Client can be run on your host system as long as you setup a shared network `--network host` when launching your container. The core implementation for it is in `./src_ur_robot_client` in the `client_core.py` file, exposing all the methods you need and websocket connection setup to the server. There are also a few example files that leverage that `client_core.py` controller in the folder. uv is used for package management and python version required is `3.10.0`. Running:
```
cd ./src_ur_robot_client
uv sync
uv run <one_of_the_example_files>
```

#### Utilities
In the `./utils` folder you can find a network configuration script that should hard-code the static ip over your available ethernet interface on a linux system, ensure the robot is configured with that static IP as well prior to attempting a direct ethernet connection.