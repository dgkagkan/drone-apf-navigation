# Docker workflow

The image contains Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, the pinned PX4
SITL checkout, Micro XRCE-DDS Agent, `px4_msgs`, the pinned Optuna tooling, and
this ROS workspace. The
default Compose service runs the complete three-drone simulation headlessly and
publishes only the dashboard on the host loopback interface.

## First run

Docker Engine with the Compose plugin is required. Copy the example settings
only when you need to change paths, IDs, or build parallelism:

```bash
cp .env.example .env
docker compose up --build
```

Open <http://127.0.0.1:8765>. Snapshots and recordings are enabled at startup
and are written to `docker-data/photos` and `docker-data/recordings`. Set
`PHOTO_DIR` and `RECORD_DIR` in `.env` to absolute host paths to store them
elsewhere.

The first image build compiles PX4, the XRCE agent, `px4_msgs`, and the project,
so it takes substantially longer than later cached builds.

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

The default uses Mesa software rendering and Gazebo EGL headless rendering so
camera and GPU LiDAR sensors work without an X server. On Intel or AMD Linux,
enable direct rendering with:

```bash
docker compose -f compose.yaml -f compose.gpu.yaml up --build
```

Set `RENDER_GID` in `.env` to the numeric host `render` group when it is not
109. For an NVIDIA host with NVIDIA Container Toolkit installed, use:

```bash
docker compose -f compose.yaml -f compose.nvidia.yaml up --build
```

To open Gazebo and RViz through X11, allow the current local user and add the
GUI overlay. It can be combined with either GPU overlay:

```bash
xhost +si:localuser:$(id -un)
docker compose -f compose.yaml -f compose.gui.yaml -f compose.gpu.yaml up --build
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
