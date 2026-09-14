# Docker workflow

The image contains Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, the pinned PX4
SITL checkout, Micro XRCE-DDS Agent, `px4_msgs`, the pinned Optuna tooling, and
this ROS workspace. The default Compose service starts Gazebo GUI, RViz, the
complete configurable drone simulation, and the dashboard on the host loopback
interface. The portable launcher automatically selects NVIDIA, Intel/AMD
`/dev/dri`, or software rendering.

## First run on Ubuntu 24.04

Docker Engine and the Compose plugin are required. First check whether they are
already available:

```bash
docker --version
docker compose version
```

If either command is missing, install Docker Engine, Buildx, and Compose from
Docker's official Ubuntu repository:

```bash
sudo apt update
sudo apt install -y ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
  -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
sudo tee /etc/apt/sources.list.d/docker.sources > /dev/null <<EOF
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")
Components: stable
Architectures: $(dpkg --print-architecture)
Signed-By: /etc/apt/keyrings/docker.asc
EOF
sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io \
  docker-buildx-plugin docker-compose-plugin
sudo systemctl enable --now docker
sudo usermod -aG docker "$USER"
newgrp docker
docker run --rm hello-world
```

If Docker is already installed but Compose is missing, run:

```bash
sudo apt update
sudo apt install -y docker-compose-plugin
```

If a visible GUI run reports `xhost: command not found`, install:

```bash
sudo apt install -y x11-xserver-utils
```

If Docker later reports permission denied for `/var/run/docker.sock`, add the
current user to Docker's group:

```bash
sudo groupadd --force docker
sudo usermod -aG docker "$USER"
```

Then log out of Ubuntu and log in again. Alternatively, run `newgrp docker` in
the current terminal. Verify the new shell before building:

```bash
id -nG                         # must contain docker
docker run --rm hello-world
```

Do not continue with `docker compose build` until `hello-world` succeeds.

The commands above follow Docker's [Ubuntu installation guide](https://docs.docker.com/engine/install/ubuntu/),
[Compose plugin guide](https://docs.docker.com/compose/install/linux/), and
[Linux post-installation guide](https://docs.docker.com/engine/install/linux-postinstall/).

After the tools are available, copy the example settings and continue in this
order:

```bash
cp .env.example .env
docker compose build
./scripts/run_docker.sh 3
```

`docker compose build` only creates the image; it does not start Gazebo, RViz, or
the dashboard. The launcher then starts the complete simulation and selects
NVIDIA, Intel/AMD, or software rendering. Its numeric argument is the number of
drones. For example, `./scripts/run_docker.sh 5` starts five vehicles.

Open <http://127.0.0.1:8765>. Snapshots and recordings are enabled at startup
and are written to `docker-data/photos` and `docker-data/recordings`. Saved
dashboard parameter profiles are written to `docker-data/settings`. Set
`PHOTO_DIR`, `RECORD_DIR`, or `SETTINGS_DIR` in `.env` to absolute host paths
to store them elsewhere.

The Settings drawer has a top-level `LIVE PARAMETERS` folder. It contains the
collapsible `PROFILES` folder and the runtime parameter folders. Save the current
values with a name, then load a profile later and press `APPLY CHANGES`.
Profiles are JSON files outside the container (`SETTINGS_DIR` on the host), so
they remain available after Docker image rebuilds and container recreation.
Loading a profile never changes a drone until the explicit apply action is
pressed.

The separate `FILES` folder controls snapshot and recording storage. Chrome and
Edge can use `CHOOSE FOLDER` to grant the browser write access to a host folder.
Firefox does not expose the required directory-write API; with Docker, use the
`SERVER PATH` fields or configure `PHOTO_DIR` and `RECORD_DIR` in `.env` before
starting the container. In a native run, the server-side picker requires
`zenity` or `kdialog`.

The first image build compiles PX4, the XRCE agent, `px4_msgs`, and the project,
so it takes substantially longer than later cached builds.

The number of simulated vehicles is configurable without changing the image:

```bash
./scripts/run_docker.sh 5
```

This starts five PX4 processes, five XRCE agents and five namespaced brains. To
choose destinations, add targets on the dashboard map, configure their altitude,
speed, and vehicle type, then press `CALCULATE` followed by `START MISSION`.
`DRONE_BASE_AGENT_PORT` changes the first agent port and
`DRONE_SPAWN_SPACING_M` changes the generated layout spacing.

## Development mode

The development overlay bind-mounts the checkout and keeps its build products
in Docker volumes:

```bash
docker compose -f compose.yaml -f compose.dev.yaml up --build
```

After changing C++ or Python code, rebuild the overlay and restart the service:

```bash
docker compose exec swarm-sim drone-dev-build
docker compose -f compose.yaml -f compose.dev.yaml restart swarm-sim
```

HTML, CSS, and JavaScript files are symlink-installed in development mode. A
browser hard refresh is normally enough after they change. Rebuild the image
when a system dependency, PX4 patch, pinned dependency commit, or Docker script
changes.

## Graphics acceleration

The base Compose service forwards the host X11 socket and defaults to software
rendering, so it can start without a GPU. `scripts/run_docker.sh` selects the
NVIDIA overlay when `nvidia-smi` detects NVIDIA, the Intel/AMD overlay when
`/dev/dri/renderD128` exists, and software rendering otherwise. NVIDIA needs a
working driver and NVIDIA Container Toolkit; Intel/AMD needs a usable render
device. Set `GPU_BACKEND=nvidia|intel|software` to override auto-detection.

For a lighter run without windows, override the two runtime flags explicitly:

```bash
DRONE_HEADLESS=true DRONE_USE_RVIZ=false ./scripts/run_docker.sh 3
```

Revoke the temporary X11 permission after stopping the stack:

```bash
xhost -si:localuser:$(id -un)
```

## Useful commands

```bash
docker compose ps
docker compose logs -f swarm-sim
docker compose exec swarm-sim bash
docker compose down
```

The published dashboard address defaults to host loopback. Change
`DASHBOARD_BIND_ADDRESS` only when another computer must reach it; the dashboard
does not currently implement authentication.
