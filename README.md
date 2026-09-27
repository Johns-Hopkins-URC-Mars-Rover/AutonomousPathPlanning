# Development Setup

Run these commands on your host machine with Docker running.

## 1. Clone the repository

```bash
git clone https://github.com/Johns-Hopkins-URC-Mars-Rover/AutonomousPathPlanning.git
cd AutonomousPathPlanning
```

## 2. Build the Docker image

```bash
docker build -t autonomous-path-planning:humble .
```

## 3. Create and enter the container

Run this from the repository root:

```bash
docker run -it --name autonomous-path-planning --mount type=bind,source="$(pwd)",target=/ros2_ws autonomous-path-planning:humble bash
```

The repository is mounted at `/ros2_ws`. Changes made inside this directory appear directly in your local repository.

For subsequent sessions, start the existing container:

```bash
docker start -ai autonomous-path-planning
```