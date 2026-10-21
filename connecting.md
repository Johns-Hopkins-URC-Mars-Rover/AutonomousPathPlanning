# Connecting a laptop to the rover's ROS 2 topics

The rover and laptop find each other through a Fast DDS **discovery server** running on the rover. Default multicast discovery is unreliable over a hotspot, so we use unicast to a known server.

## Setup

### On the rover (`jhurover`)

Start the server in its own terminal or tmux session. It must be running before the rover's nodes start.

```bash
source /opt/ros/humble/setup.bash
fastdds discovery --server-id 0 --udp-port 11811
```

Start the rover's nodes from a shell with:

```bash
export ROS_DOMAIN_ID=42
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
```

### On the laptop

Run this in every new terminal (variables don't persist):

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export ROS_DISCOVERY_SERVER="<rover-ip>:11811"   # e.g. 172.20.10.9
export ROS_SUPER_CLIENT=true                     # lets `ros2 topic list` see all topics
ros2 daemon stop
ros2 topic list
```

The rover's IP comes from the hotspot and can change. Check it with `hostname -I` on the rover.

## What went wrong (symptom: only `/parameter_events` and `/rosout`)

There were three separate causes. Any one of them gives the same symptom.

1. **The discovery server wasn't running on the rover.** `ps aux | grep fast-discovery` and `ss -ulnp | grep 11811` on the rover showed nothing. Note `nc -zvu <ip> 11811` from the laptop reports "succeeded" for UDP even when nothing is listening, so it proves nothing.
   - Also: `fastdds discovery --port` is not a valid flag. Use `--udp-port` (or `-p`).
   - If the command says "fast-discovery-server tool not found", source ROS first, or `sudo apt install ros-humble-fastrtps`.
2. **The exports were missing in the terminal being used.** A new terminal has none of them, and without `ROS_DISCOVERY_SERVER` the laptop silently falls back to default discovery. Run `env | grep ROS` to confirm all four variables are set.
3. **Rover nodes started without the exports** don't register with the server. Restart them from a shell that has `ROS_DOMAIN_ID` and `ROS_DISCOVERY_SERVER` set, after the server is up.

## Debugging checklist

Work from the rover outward.

1. **Rover:** is the server up? `ps aux | grep fast-discovery` and `ss -ulnp | grep 11811`.
2. **Rover:** with the exports and `ROS_SUPER_CLIENT=true`, does `ros2 topic list` show the rover's own topics? If not, the nodes aren't registered with the server.
3. **Laptop:** `env | grep ROS` should show domain 42, the rover's IP, and `ROS_SUPER_CLIENT=true`.
4. **Laptop:** try `ros2 topic list --no-daemon` to rule out a stale daemon cache. After changing variables, always `ros2 daemon stop`.
5. **Network:** `ping <rover-ip>`, then `sudo tcpdump -ni any host <laptop-ip>` on the rover (`sudo apt install tcpdump` if missing) while running `ros2 topic list` on the laptop. Packets from the laptop should arrive. Don't filter on `udp port 11811` only: the server replies from other ports.
6. Both sides must use the same RMW (Fast DDS is the Humble default). CycloneDDS ignores `ROS_DISCOVERY_SERVER`.

## Notes

- Listing topics only needs the server. Receiving data (`ros2 topic echo`) needs a direct UDP path between the machines.
- Over a hotspot, prefer the `/compressed` image topics in rviz. The raw images and point cloud are large.
- VS Code installed as a snap leaks environment variables that crash rviz2 (`GSettings ... 'antialiasing'`). Run rviz2 from a regular terminal, or with `GSETTINGS_SCHEMA_DIR`, `GTK_PATH`, `GTK_EXE_PREFIX`, `GIO_MODULE_DIR`, `GTK_IM_MODULE_FILE` and `LOCPATH` unset and `XDG_DATA_DIRS=/usr/share/ubuntu:/usr/local/share/:/usr/share/:/var/lib/snapd/desktop`.
