# isar-taurob

Integration and Supervisory control of Autonomous ground Robots - Taurob

## Installation

Clone and install in the repository root folder.

```bash
git clone https://github.com/equinor/isar-taurob
cd isar-taurob
uv sync --extra dev
```

For development, it is recommended to install the latest version of ISAR from [GitHub](https://github.com/equinor/isar) locally.

Finally, verify that you can run the tests:

```bash
uv run pytest .
```

## Dependencies

The dependencies used for this package are listed in `pyproject.toml` and pinned in `uv.lock`. This ensures our builds are predictable and deterministic. This project uses [uv](https://docs.astral.sh/uv/) for dependency management:

Install local versions (editable):
```
uv sync
uv pip install -e ".[dev]"
```

Install pinned versions:
```
uv sync --no-sources
```

Pin local version:
```
uv lock --no-sources
```

To update the dependencies to the latest versions, run:

```
uv lock --upgrade --no-sources
```


### Alitra setup

Alitra is used to transform between asset and robot_frame. To align these it is adviced to confirm mapping using `eq_robot_utility_scripts/flotilla_mission_scripts/main_check_alitra`

1. Run the script to get the robot map.
2. Find 4 coordinates in both Echo and in the robot_map & lower and upper bounds; Fill these into the corresponding 'alitra_transform(...).json' file
3. Ensure there is no overlap with existing maps.
4. Run the script again to verify that the alitra mapping is good. Printed errors should be lower than 10 cm, ideally lower than 3-5 cm.
5. If not happy - move some points, or add more coordinates.
6. When satisfied, the transform has been confirmed. Move the json file into `isar-taurob`.

# Configuring Azure resources for the Taurob cloud, and running locally with taurob cloud
See here: https://github.com/equinor/robotics-infrastructure/blob/main/automation/taurob/README.md
