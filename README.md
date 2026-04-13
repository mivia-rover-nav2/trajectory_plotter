# Trajectory Plotter

A Python utility for generating and saving 2D trajectories (with orientation quaternions) for mobile robots or similar applications.

## Requirements

- Python 3.10+
- `numpy`
- `numpy-quaternion`
- `matplotlib`

Install dependencies with:

```bash
pip install numpy numpy-quaternion matplotlib
```

## Usage

Run the script and follow the interactive menu:

```bash
python generate_traj.py
```

You will be prompted to choose a trajectory type and enter its parameters (default values are shown in brackets — press Enter to accept them).

## Available Trajectories

| # | Type | Parameters |
|---|------|------------|
| 1 | **Circular** | radius, number of points |
| 2 | **Linear** | start point (x, y), end point (x, y), number of points |
| 3 | **Rectangular** | bottom-left corner (x, y), width, height, points per side |
| 4 | **Rectangular with rounded corners** | bottom-left corner (x, y), width, height, corner radius, points per side, points per corner arc |

## Output

Each trajectory is:
- **Plotted** in a matplotlib window.
- **Saved** to a `.csv` file in the current directory.

The CSV format is one waypoint per line:

```
x,y,qx,qy,qz,qw
```

where `(qx, qy, qz, qw)` is the orientation quaternion (rotation around the Z-axis, representing heading).

## Project Structure

```
trajectory_plotter/
├── generate_traj.py   # Main script — trajectory generation, plotting, saving
└── README.md
```
