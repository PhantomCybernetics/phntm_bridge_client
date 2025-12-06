# Phantom Bridge Client

Fast WebRTC + Socket.io ROS2 Bridge for real-time data and video streaming, teleoperation, HRI, and remote robot monitoring.
Comes with Docker Container control for the host machine, CPU and Wi-Fi monitoring, and customizable [Web Interface](https://docs.phntm.io/bridge/ui/overview.html). \
\
[See full documentation here](https://docs.phntm.io/bridge)

## Features
- ROS2 Node, Topic and Service discovery
- Fast streamimg of binary ROS2 messages (in a out)
- Fast H.264 video streaming (pre-encodeded FFmpeg frames)
- Image & CompressesImage messages encoded and streamed as H.264 video (sw, cuda or vaapi encoding)
- Docker container discovery and control
- Reliable ROS2 Service calls via Socket.io
- ROS2 runtime Parameneters read/write API
- Extra ROS2 packages can be easily included for custom message and service type support
- Robot's Wi-Fi signal monitoring, scan & roaming (via Agent, requires wpa_supplicant on the host machine)
- File retreival from any running Docker container (such as URDF meshes, via Agent) 
- System load and Docker stats monitoring (via Agent)
- Connects P2P or via a TURN server when P2P link is not possible
- Multiple peers can connect to the same machine at a very low extra CPU cost
- ~5-10ms RTT on local network, 50ms+ RTT remote operation via a TURN server
- Works with Rosbag and Sims such as Gazebo or Webots
- Supported with ROS2 Humble and newer

## Architecture
![Infrastructure map](https://raw.githubusercontent.com/PhantomCybernetics/phntm_bridge_docs/refs/heads/main/img/Architecture_Bridge.png)

## Install

> [!WARNING]
> This is a work in progress, and some radical changes are still taking place.
> Please wait until the first official release before installing this package,
> as the cloud infrastructure may be down or undergoing maintenance at any time.
> Some breaking changes may still occur, requiring a full re-install. 
> The best way to get notified about a stable release is to use the Watch feature here on GitHub,
> or by following [@phntm.io](https://bsky.app/profile/phntm.io) on BlueSky.

### Make sure your root SSL Certificates are up to date

```bash
sudo apt update
sudo apt install ca-certificates
```

### Install Docker, Docker Build & Docker Compose

E.g. on Debian/Ubuntu follow [these instructions](https://docs.docker.com/engine/install/debian/). Then add the current user to the docker group:
```bash
sudo usermod -aG docker ${USER}
# log out & back in
```

### (Optional) Clone this repo and build the Docker image from source

You can also use our pre-built Docker images, see [ghcr.io/phantomcybernetics/phntm_bridge_client](https://github.com/PhantomCybernetics/phntm_bridge_client/pkgs/container/phntm_bridge_client) for ROS distributions and architectures.

```bash
cd ~
git clone git@github.com:PhantomCybernetics/phntm_bridge_client.git phntm_bridge_client
cd phntm_bridge_client
ROS_DISTRO=humble; docker build -f Dockerfile -t phntm/bridge:$ROS_DISTRO --build-arg ROS_DISTRO=$ROS_DISTRO .
```

### Register a new Robot on the Bridge Server
This registers a new robot on the Bridge Server and returns default config file you can edit further. Unique ID_ROBOT and KEY pair are generated in this step.
```bash
wget -O ~/phntm_bridge.yaml 'https://register.phntm.io/robot?yaml'
```

### Examine and customize the config file
Below is an example of the config file generated in the previous step, e.g. `~/phntm_bridge.yaml`. \
Full list of configuration options can be found [here](https://docs.phntm.io/bridge/basics/configuration.html).
```yaml
/**:
  ros__parameters:
    id_robot: '%ID_ROBOT%'
    key: '%SECRET_KEY%'
    name: 'Unnamed Robot'
    maintainer_email: 'robot.master@example.com' # e-mail for service announcements

    bridge_server_address: https://us-ca.bridge.phntm.io

    log_sdp: True # verbose WebRTC debug
    log_heartbeat: True # debug heartbeat

    ## Introspection
    discovery_period_sec: 3.0 # < 0 introspection OFF
    stop_discovery_after_sec: 10.0 # < 0 run forever

    ## Extra packages to install, this is either a package folder mounted into the container,
    ## or a ROS2 package name to be installed via apt-get (for e.g. "ros-humble-some-package" use only "some-package")
    extra_packages:
      - /ros2_ws/src/vision_msgs
      - some-package

    ## Blink LEDs via GPIO on network activity
    conn_led_gpio_chip: /dev/gpiochip0
    conn_led_pin: 23
    data_led_pin: 24

    ## Custom topic configs
    /rosout: # TODO add to default config
      reliability: RELIABLE
      durability: TRANSIENT_LOCAL
      history_depth: 20
    /robot_description:
      reliability: RELIABLE
      durability: TRANSIENT_LOCAL
    /tf_static:
      reliability: RELIABLE
      durability: TRANSIENT_LOCAL
    /battery:
      min_voltage: 19.2 # set empty voltage
      max_voltage: 25.2 # set set full voltage

    ui_battery_topic: /battery # battery to show in the UI, '' to disable

    wifi_interface: 'wlan0'
    wifi_monitor_topic: /iw_status # WiFi monitor topic to show in the UI (produced by the Agent)
    enable_wifi_scan: True
    enable_wifi_roam: False

    docker_monitor_topic: /docker_info # produced by the Agent
    enable_docker_control: True # Docker control via Agent

    ## User input config
    input_drivers: [ 'TwistInputDriver', 'JoyInputDriver' ] # enabled input drivers
    input_defaults: /ros2_ws/phntm_input_config.json # path to input config file as mapped inside the container
    service_defaults: /ros2_ws/phntm_service_config.json # path to services config file as mapped inside the container
```

### Add service to your compose.yaml

> [!IMPORTANT]
> We recommend using Cyclone DDS with this Bridge as it offers a more predictable behavior,
> the default Fast DDS sometimes fails to receive messages for topics that were unsubscribed and subscrubed to again.
> Cyclone DDS is installed with our Docker image, and selected with the ``RMW_IMPLEMENTATION`` environmental variable.
> All parts of your ROS2 system [should be using the same ROS version and the same RMW implementation](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Different-Middleware-Vendors.html).

Add phntm_bridge service to your `~/compose.yaml` file with both `~/phntm_bridge.yaml` and `~/phntm_agent.yaml` mounted in the container as shown below:
```yaml
services:
  phntm_bridge:
    image: ghcr.io/phantomcybernetics/phntm_bridge_client:main-jazzy # or phntm/bridge:$ROS_DISTRO if image is built locally
    container_name: phntm-bridge
    hostname: phntm-bridge.local
    restart: unless-stopped # restarts after first run
    privileged: true # bridge needs this
    # cpuset: '0,1,2' # consider dedicating a few CPU cores
    network_mode: host # webrtc needs this
    ipc: host # bridge needs this to see other local containers
    # environment:
    #  - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp # recommended, see the note above!
    #  - ROS_DOMAIN_ID=22 # if used, specify ROS domain ID here
    volumes:
      # - ~/phntm_bridge_client:/ros2_ws/src/phntm_bridge # (optional) live repo mapped here for easy updates
      - ~/phntm_bridge.yaml:/ros2_ws/phntm_bridge_params.yaml # bridge config goes here
      - ~/phntm_bridge.yaml:/ros2_ws/phntm_agent_params.yaml # agent config goes here, can be shared with the bridge client config
      - /var/run:/host_run # docker file extractor and wifi control need this
      - /tmp:/tmp # wifi control needs this
    devices:
      - /dev:/dev # LED control needs this
    command:
      ros2 launch phntm_bridge client_agent_launch.py # launches Bridge Client and Agent together
```

### Launch
```bash
docker compose up phntm_bridge # launches Bridge Client & Agent in one container
```

### Open the Web UI
Navigate to `https://bridge.phntm.io/%YOUR_ID_ROBOT%` in a web browser. The exact link can be found at the top of the generated Bridge config file (e.g. your `~/phntm_bridge.yaml`). If you provided maintainer's e-mail in the config, it will be also e-mailed to you for your reference after the first Bridge Client launch.

## Upgrading
```bash
# Remove previous version
docker stop phntm-bridge && docker rm phntm-bridge && docker image rm phntm/bridge:humble

# if using an image
docker compose pull phntm_bridge

# or update & rebuild from source
cd ~/phntm_bridge_client
git pull
ROS_DISTRO=humble; docker build -f Dockerfile -t phntm/bridge:$ROS_DISTRO --build-arg ROS_DISTRO=$ROS_DISTRO .

# Launch
docker compose up phntm_bridge
```

## See also
- [Documentation](https://docs.phntm.io/bridge) Full Phantom Bridge documentation
- [Bridge UI](https://docs.phntm.io/bridge/ui/overview.html) Overview of the customizable web UI
- [Configure User Input](https://docs.phntm.io/bridge/ui/user-input-and-teleoperation.html) Use keyboard, touch interface or gamepad to control your robot locally or remotely
- [Picam ROS2](https://github.com/PhantomCybernetics/picam_ros2) Standalone ROS2 node that converts hardware-encoded H.264 frames into ROS messages
