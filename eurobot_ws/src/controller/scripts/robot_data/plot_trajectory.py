import matplotlib.pyplot as plt
import csv

x_vals, y_vals = [], []

with open('trajectory_02.csv', 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        x_vals.append(float(row['x']))
        y_vals.append(float(row['y']))

plt.figure()
plt.plot(x_vals, y_vals, marker='o')
plt.title("Trajectòria del robot")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.axis('equal')
plt.grid(True)
plt.show()