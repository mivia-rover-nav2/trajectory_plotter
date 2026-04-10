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
    
    from generate_traj import TrajectoryGenerator

    traj_gen = TrajectoryGenerator()
    #traj = traj_gen.generate_circular_trajectory(radius=1, num_points=1000)
    traj = traj_gen.generate_linear_trajectory(Waypoint(0, 0, quaternion.from_rotation_vector([0, 0, 0])), Waypoint(0, 2, quaternion.from_rotation_vector([0, 0, 0])), num_points=100)
    # traj_gen.plot_trajectory(traj)
    transfomed_traj = traj_gen.change_origin(traj, Waypoint(2.0, 3.0, quaternion.from_rotation_vector([0, 0, 0])))
    traj_gen.plot_trajectory(transfomed_traj)
    traj_gen.save_trajectory("trajectory_linear.csv", transfomed_traj)

    plt.show()