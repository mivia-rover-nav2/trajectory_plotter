import matplotlib.pyplot as plt
import numpy as np
import quaternion

class Waypoint:
    def __init__(self, x, y, quaternion):
        self.x = x
        self.y = y
        self.quaternion = quaternion
    
    def __str__(self):
        return f"{self.x},{self.y},{self.quaternion.x},{self.quaternion.y},{self.quaternion.z},{self.quaternion.w}"

class TrajectoryGenerator:
    def __init__(self):
        pass
        

    def generate_circular_trajectory(self, radius:float, num_points:int=100) -> list[Waypoint]:
        # Generate a trajectory. The trajectory must be a list of waypoints made of x,y,quaternions.       
        # the trajectory is made from the equation x^2+y^2 = radius^2
        trajectory = []
        for i in range(num_points):
            angle = 2 * np.pi * i / num_points
            x = radius * np.cos(angle)
            y = radius * np.sin(angle)
            quat = quaternion.from_rotation_vector([0, 0, angle])  # Simple rotation around Z-axis
            waypoint = Waypoint(x, y, quat)
            trajectory.append(waypoint)
        return trajectory
    
    def generate_linear_trajectory(self, start:Waypoint, end:Waypoint, num_points:int=100) -> list[Waypoint]:
        # The heading is determined by the direction of the segment, constant along the whole trajectory
        # NOTE: since the trajectory is linear, the heading is constant and is determined by the direction of the segment connecting the start and end points. The quaternion for each waypoint will be the same, representing this constant heading.
        trajectory = []
        heading = np.arctan2(end.y - start.y, end.x - start.x)
        quat = quaternion.from_rotation_vector([0, 0, heading])
        for i in range(num_points):
            t = i / (num_points - 1)  # Normalized parameter from 0 to 1
            x = (1 - t) * start.x + t * end.x
            y = (1 - t) * start.y + t * end.y
            trajectory.append(Waypoint(x, y, quat))
        return trajectory
    
    def generate_rectangular_trajectory(self,bottom_left_point:tuple ,width:float, height:float, num_points_per_side:int=25) -> list[Waypoint]:
        trajectory = []
        # Define the corners of the rectangle
        corners = [
            Waypoint(bottom_left_point[0], bottom_left_point[1], quaternion.from_rotation_vector([0, 0, 0])),  # Bottom-left
            Waypoint(bottom_left_point[0] + width, bottom_left_point[1], quaternion.from_rotation_vector([0, 0, np.pi/2])),  # Bottom-right
            Waypoint(bottom_left_point[0] + width, bottom_left_point[1]+ height, quaternion.from_rotation_vector([0, 0, np.pi])),  # Top-right
            Waypoint(bottom_left_point[0], bottom_left_point[0]+ height, quaternion.from_rotation_vector([0, 0, -np.pi/2]))  # Top-left
        ]
        
        # Generate linear trajectories for each side of the rectangle
        for i in range(4):
            start = corners[i]
            end = corners[(i + 1) % 4]  # Wrap around to the first corner
            trajectory.extend(self.generate_linear_trajectory(start, end, num_points_per_side))
        return trajectory


    def generate_rectangular_trajectory_smoothed(self, bottom_left_point: tuple, width: float, height: float,
                                                   corner_radius: float, num_points_per_side: int = 25,
                                                   num_points_per_corner: int = 10) -> list[Waypoint]:
        """Rectangular trajectory with quarter-circle rounded corners.

        The rectangle is traversed counter-clockwise (east → north → west → south).
        Each corner is replaced by a CCW quarter-circle arc of the given radius.
        corner_radius is automatically clamped to min(width, height) / 2.
        """
        bx, by = bottom_left_point
        r = min(corner_radius, width / 2, height / 2)

        def _line(x1, y1, x2, y2, heading, n):
            if x1 == x2 and y1 == y2:
                return []
            quat = quaternion.from_rotation_vector([0, 0, heading])
            return [
                Waypoint(
                    (1 - t) * x1 + t * x2,
                    (1 - t) * y1 + t * y2,
                    quat
                )
                for t in np.linspace(0, 1, n, endpoint=False)
            ]

        def _arc(cx, cy, start_angle, end_angle, n):
            # CCW arc: tangent heading = angle + π/2
            return [
                Waypoint(
                    cx + r * np.cos(a),
                    cy + r * np.sin(a),
                    quaternion.from_rotation_vector([0, 0, a + np.pi / 2])
                )
                for a in np.linspace(start_angle, end_angle, n, endpoint=False)
            ]

        trajectory = []
        # Bottom side — going east (heading 0)
        trajectory += _line(bx + r, by, bx + width - r, by, 0.0, num_points_per_side)
        # Bottom-right arc: center (bx+w-r, by+r), -π/2 → 0
        trajectory += _arc(bx + width - r, by + r, -np.pi / 2, 0, num_points_per_corner)
        # Right side — going north (heading π/2)
        trajectory += _line(bx + width, by + r, bx + width, by + height - r, np.pi / 2, num_points_per_side)
        # Top-right arc: center (bx+w-r, by+h-r), 0 → π/2
        trajectory += _arc(bx + width - r, by + height - r, 0, np.pi / 2, num_points_per_corner)
        # Top side — going west (heading π)
        trajectory += _line(bx + width - r, by + height, bx + r, by + height, np.pi, num_points_per_side)
        # Top-left arc: center (bx+r, by+h-r), π/2 → π
        trajectory += _arc(bx + r, by + height - r, np.pi / 2, np.pi, num_points_per_corner)
        # Left side — going south (heading -π/2)
        trajectory += _line(bx, by + height - r, bx, by + r, -np.pi / 2, num_points_per_side)
        # Bottom-left arc: center (bx+r, by+r), π → 3π/2
        trajectory += _arc(bx + r, by + r, np.pi, 3 * np.pi / 2, num_points_per_corner)

        return trajectory

    def plot_trajectory(self, trajectory:list[Waypoint]):
        for waypoint in trajectory:
            
            plt.plot(waypoint.x,waypoint.y, marker='o')
            plt.title('Generated Trajectory')
            ax = plt.gca()
            ax.set_aspect('equal', adjustable='box')
            plt.xlabel('X-axis')
            plt.ylabel('Y-axis')
    
    def save_trajectory(self, filename, trajectory:list[Waypoint]):
        with open(filename, 'w') as f:
            for point in trajectory:
                f.write(f"{point}\n")
    
    def change_origin(self, trajectory:list[Waypoint], new_origin:Waypoint) -> list[Waypoint]:
        # Change the origin of the trajectory to the new origin
        # la trasformazione deve essere fatto considerando che dal sistema attuale sto passando a quello nuovo
        print("The old origin is now the point ({:.2f}, {:.2f})".format(new_origin.x, new_origin.y))
        transformed_trajectory = []
        for waypoint in trajectory:
            # Translate the waypoint to the new origin
            x_translated = waypoint.x + new_origin.x
            y_translated = waypoint.y + new_origin.y
            
            # Rotate the waypoint by the inverse of the new origin's quaternion
            quat_conjugate = np.conjugate(new_origin.quaternion)
            rotated_quat = quat_conjugate * waypoint.quaternion
            
            transformed_waypoint = Waypoint(x_translated, y_translated, rotated_quat)
            transformed_trajectory.append(transformed_waypoint)
        return transformed_trajectory

if __name__ == "__main__":

    TRAJECTORIES = {
        "1": "Circular",
        "2": "Linear",
        "3": "Rectangular",
        "4": "Rectangular (rounded corners)",
    }

    print("Available trajectories:")
    for key, name in TRAJECTORIES.items():
        print(f"  {key}. {name}")

    choice = input("Select a trajectory [1-4]: ").strip()

    if choice not in TRAJECTORIES:
        print(f"Invalid choice '{choice}'. Exiting.")
        exit(1)

    traj_gen = TrajectoryGenerator()

    if choice == "1":
        radius = float(input("Radius [default 1.0]: ") or "1.0")
        num_points = int(input("Number of points [default 100]: ") or "100")
        traj = traj_gen.generate_circular_trajectory(radius=radius, num_points=num_points)
        filename = f"trajectory_circular_r{radius}.csv"

    elif choice == "2":
        print("Start point:")
        x0 = float(input("  x [default 0.0]: ") or "0.0")
        y0 = float(input("  y [default 0.0]: ") or "0.0")
        print("End point:")
        x1 = float(input("  x [default 1.0]: ") or "1.0")
        y1 = float(input("  y [default 0.0]: ") or "0.0")
        num_points = int(input("Number of points [default 100]: ") or "100")
        start = Waypoint(x0, y0, quaternion.from_rotation_vector([0, 0, 0]))
        end   = Waypoint(x1, y1, quaternion.from_rotation_vector([0, 0, 0]))
        traj = traj_gen.generate_linear_trajectory(start, end, num_points=num_points)
        filename = "trajectory_linear.csv"

    elif choice == "3":
        bx = float(input("Bottom-left x [default 0.0]: ") or "0.0")
        by = float(input("Bottom-left y [default 0.0]: ") or "0.0")
        width  = float(input("Width  [default 2.0]: ") or "2.0")
        height = float(input("Height [default 2.0]: ") or "2.0")
        num_pts = int(input("Points per side [default 25]: ") or "25")
        traj = traj_gen.generate_rectangular_trajectory((bx, by), width=width, height=height, num_points_per_side=num_pts)
        filename = "trajectory_rectangular.csv"

    else:  # choice == "4"
        bx = float(input("Bottom-left x [default 0.0]: ") or "0.0")
        by = float(input("Bottom-left y [default 0.0]: ") or "0.0")
        width   = float(input("Width  [default 2.0]: ") or "2.0")
        height  = float(input("Height [default 2.0]: ") or "2.0")
        radius  = float(input("Corner radius [default 0.3]: ") or "0.3")
        num_pts = int(input("Points per side   [default 25]: ") or "25")
        num_cor = int(input("Points per corner [default 10]: ") or "10")
        traj = traj_gen.generate_rectangular_trajectory_smoothed(
            (bx, by), width=width, height=height,
            corner_radius=radius, num_points_per_side=num_pts, num_points_per_corner=num_cor
        )
        filename = "trajectory_rectangular_smoothed.csv"

    traj_gen.plot_trajectory(traj)
    traj_gen.save_trajectory(filename, traj)
    print(f"Trajectory saved to '{filename}'.")

    plt.show()