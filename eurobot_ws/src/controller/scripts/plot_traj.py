import csv
import matplotlib.pyplot as plt
import numpy as np

class DifferentialWheel:

    def __init__(self, l=0.21, radius=0.0475, x_init=0, y_init=0, theta_init=0, max_v=1, max_w=2):
        """
        Initializes all the attributes of the class.

        Args:
            l (double): The robot longitude (from back to front) expressed in meters.
            radius (double): The wheels radius expressed in meters.
            x_init (integer): Initial robot x coordinate.
            y_init (integer): Initial robot y coordinate.
            theta_init (integer): Initial robot orientation.
            max_vel (integer): Maximum linear velocity the robot can reach.
            max_w (integer): Maximum angular velocity the robot can reach.
        """
        
        self.l      = l  
        self.radius = radius
        self.x      = x_init
        self.y      = y_init
        self.theta  = theta_init
        self.max_v  = max_v
        self.max_w  = max_w


    def get_velocities(self, wl, wr):
        """
        Calculates the robot's linear and angular velocities based on the wheel velocities

        Args:
            wl (float): Left wheel velocity in radians per second.
            wr (float): Right wheel velocity in radians per second.

        Returns:
            tuple: The linear velocity (v) in meters per second and the angular velocity (w) in radians per second.
        """
        
        vl = wl * self.radius
        vr = wr * self.radius
        
        w = (vr - vl) / self.l
        v = (vr + vl) / 2

        return v, w
    

    def get_motor_velocities(self, v, w):
        """
        Calculates the wheel velocities based on the robot's linear and angular velocities.

        Args:
            v (float): Linear velocity in meters per second.
            w (float): Angular velocity in radians per second.

        Returns:
            tuple: The left wheel velocity (vl) and right wheel velocity (vr) in meters per second.
        """

        vl = (float)(v - (w * self.l) / 2)
        vr = (float)(v + (w * self.l) / 2)

        return vl, vr
    

    def integrate_velocity(self, v, w, inc_t):
        """
        Updates the robot's position and orientation by integrating its linear and angular velocities over time.

        Args:
            v (float): Linear velocity in meters per second.
            w (float): Angular velocity in radians per second.
            inc_t (float): Time increment in seconds.

        Returns:
            tuple: The linear velocity (v) and angular velocity (w) after applying the velocity limits.
        """

        # Check if the parameters exceed the maximum velocities and if that's the case correct it
        if v > self.max_v:
            v = self.max_v

        if w > self.max_w:
            w = self.max_w

        # Calculate the current position from the matrices calculations
        mat_01 = np.array([[self.x], [self.y], [self.theta]])
        mat_02 = np.array([[np.cos(self.theta), (-np.sin(self.theta)), 0], [np.sin(self.theta), np.cos(self.theta), 0], [0, 0, 1]])
        mat_03 = np.array([[v], [0], [w]])

        mat_resultat = mat_01 + mat_02 @ mat_03 * inc_t

        self.x = mat_resultat[0][0]
        self.y = mat_resultat[1][0]
        self.theta = mat_resultat[2][0]

        return v, w


    def update_state(self, wl, wr, inc_t):
        """
        Updates the robot's state by calculating its linear and angular velocities and integrating them over time.

        Args:
            wl (float): Linear velocity in meters per second.
            wr (float): Angular velocity in radians per second.
            inc_t (float): Time increment in seconds.

        Returns:
            tuple: The linear velocity (v) and angular velocity (w) after applying the velocity limits.
        """
        
        v, w = self.get_velocities(wl, wr)
        self.integrate_velocity(v, w, inc_t)
        
        return v, w
    

    def get_state(self):
        """
        Returns the current state of the robot as a numpy array.

        Returns:
            numpy.array: A numpy array containing the robot's position (x, y) and orientation (theta).
        """

        return np.array([self.x, self.y, self.theta])
    

    def set_theta(self, theta_from_gyro):
        """
        Updates the robot's orientation using an external sensor.
        """

        self.theta = theta_from_gyro
    
    
    def __str__(self):
        """
        Returns a string representation of the robot's current state.

        Returns:
            string: A formatted string displaying the robot's position (x, y) in centimeters and orientation (theta) in both radians and degrees.
        """

        return (
            f"Robot State:\n"
            f"Position:\n"
            f"  x: {self.x:.2f} cm\n"
            f"  y: {self.y:.2f} cm\n"
            f"Orientation:\n"
            f"  theta: {self.theta:.2f} radians\n"
            f"  theta (degrees): {np.degrees(self.theta):.2f}°"
        )

def read_csv(file_path):
    timestamps = []
    left_enc = []
    right_enc = []
    with open(file_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            timestamps.append(float(row['timestamp']))
            left_enc.append(float(row['left_encoder_cm']) / 100.0)  # cm -> m
            right_enc.append(float(row['right_encoder_cm']) / 100.0)
    return timestamps, left_enc, right_enc

def compute_angular_velocity(encoder_positions, r, dt):
    return [(encoder_positions[i+1] - encoder_positions[i]) / (r * dt[i]) for i in range(len(dt))]

def plot_trajectory(x, y):
    plt.plot(x, y, marker='o')
    plt.title("Trajectòria del robot")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.axis("equal")
    plt.grid(True)
    plt.show()

def main():
    file_path = 'odometry_01.csv'  # Canvia-ho al teu fitxer
    timestamps, left_pos, right_pos = read_csv(file_path)

    dt = [timestamps[i+1] - timestamps[i] for i in range(len(timestamps)-1)]
    drive = DifferentialWheel()

    # Calcular velocitats angulars de roda (rad/s)
    wl = compute_angular_velocity(left_pos, drive.r, dt)
    wr = compute_angular_velocity(right_pos, drive.r, dt)

    input("Prem ENTER per començar la simulació...")

    for i in range(len(dt)):
        drive.update_state(wl[i], wr[i], dt[i])

    plot_trajectory(drive.x_history, drive.y_history)

if __name__ == "__main__":
    main()
