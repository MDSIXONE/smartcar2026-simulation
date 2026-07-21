# 2026-07-18 Portable Gazebo assets

## Purpose

Make Task 3 cubes and area-label meshes load in a local WSL Ubuntu 20.04
workspace instead of relying on the original developer's absolute path.

## Changes

- Cube visual URIs in `src/car3/models/cube/model_0.sdf` through
  `model_2.sdf` now use `model://cube/meshes/...`.
- `src/car3/launch/gazebo.launch` sets `GAZEBO_MODEL_PATH` before
  Gazebo starts, so `gzserver` receives the project model directory.
- Wall labels in `src/car3/world/math.world` now use
  `model://sign/meshes/...`.
- `src/car3/models/sign/model.config` and `model.sdf` make the sign mesh
  directory a valid Gazebo model resource.
- `test_cube_mesh_uri.py` and `test_sign_mesh_uri.py` prevent a regression to
  absolute, machine-specific mesh URIs.

## Verification

On local WSL Ubuntu 20.04, the following checks passed after the files were
synchronized into `~/smartcar2026-simulation`:

```bash
python3 src/car3/test/test_cube_mesh_uri.py
python3 src/car3/test/test_sign_mesh_uri.py
```

`task3_prepare.launch gui:=true rviz:=true` was restarted against the local
Master at `http://192.168.8.197:11311`.  Gazebo spawned the cube entities and
reported no missing sign-mesh URI error.  Final visual verification remains
camera-dependent: zoom or rotate the Gazebo view if a label is outside the
current frame.

## Limitation

These resource-only changes do not require a rebuild after a workspace has
already been built; Gazebo reads the world, launch, and model files at startup.
An initial clone or any compiled-code change still requires `catkin_make`.
