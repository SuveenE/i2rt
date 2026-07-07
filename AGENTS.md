# AGENTS.md

## Cursor Cloud specific instructions

`i2rt` is a Python library for controlling I2RT robots (YAM arm, Flow base) over
a CAN bus, plus MuJoCo-based kinematics/visualization. See `README.md` for the
full API and hardware workflows.

### Environment
- Python **3.12** in a `uv`-managed virtualenv at `.venv/` (created by the
  startup update script). Activate with `source .venv/bin/activate`, or call
  tools directly via `.venv/bin/...`.
- `uv` lives at `~/.local/bin/uv`. System build/GL libraries
  (`build-essential`, `python3.12-dev`, `libosmesa6`, `libegl1`, mesa drivers)
  are preinstalled in the snapshot.
- `ruckig==0.15.3` only ships an sdist and must be built from source. Its build
  breaks with `scikit-build-core >= 0.10`, so installs use a build constraint
  (`scikit-build-core<0.10`). The update script already handles this — if you
  reinstall deps manually, pass `--build-constraint` with that pin.

### Lint / test / run
- **Lint:** `ruff check .` (repo pins `ruff==0.6.8` in `.pre-commit-config.yaml`;
  the newer default ruff reports extra findings). Note: `ruff check .` currently
  reports pre-existing violations in `i2rt/flow_base/*` and `scripts/*` on a
  clean checkout — these are not introduced by your changes.
- **Tests:** `pytest i2rt/` runs the real unit suite (MuJoCo FK/IK, no hardware).
  Running bare `pytest` from the root also collects `scripts/test_gamepad.py`,
  whose `test_hid_gamepad` requires a physical gamepad + the optional `hid`
  module and will fail without hardware — this is expected, not a setup bug.
- **Run (no hardware):** MuJoCo kinematics/visualization work headlessly.
  `python i2rt/robots/kinematics.py` prints an FK/IK demo. Most other entry
  points (`scripts/minimum_gello.py`, `i2rt/flow_base/*`) need CAN hardware.

### MuJoCo rendering gotcha
- Headless offscreen rendering requires `MUJOCO_GL=osmesa` (software). `egl`
  fails in this VM. The interactive `mujoco.viewer.launch_passive` viewer needs
  a display and is not usable headlessly.
- The offscreen framebuffer defaults to 640x480; larger `Renderer` sizes need
  `<visual><global offwidth=.../></visual>` in the model XML.
