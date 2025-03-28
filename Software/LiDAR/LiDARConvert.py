import pandas as py
import numpy as np
import matplotlib.pyplot as plt

def cartesian_to_polar(x, y): 
    r = np.sqrt(np.square(x) + np.square(y))
    theta = np.arctan2(y, x)
    return r, theta

def polar_to_cartesian(r, theta):
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    return x, y

def spherical_to_cartesian(r, theta, phi):
    """
    Converts spherical vectors (3d polar) into 3d cartesian vectors

    Parameters: 
    r : radius of the vector

    theta: angle in radians between x axis and vector

    phi: angle in radians between z axis and vector

    returns vectors of x, y, and z
    """

    x = r * np.sin(theta) * np.cos(phi)
    y = r * np.sin(theta) * np.sin(phi)
    z = r * np.cos(theta)

    return x, y, z

def cartesian_to_spherical(x, y, z):
    """
    Converts spherical vectors (3d polar) into 3d cartesian vectors

    Parameters: 
    x : x vector

    y: y vector

    z: z vector

    returns vectors of the radius, the theta and phi angles
    """
    r = np.sqrt(x ** 2 + y ** 2 + z ** 2)
    theta = np.arccos(z / r)
    phi = np.arctan(y / x)


    return r, theta, phi

def plot_polar_points(r_points, theta_points):
    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})

    ax.plot(theta_points, r_points)
    ax.grid(True)
    ax.set_title("A line plot on a polar axis", va='bottom')

    plt.show()

def plot_cartesian_points(x_points, y_points):
    plt.plot(x_points, y_points)

    plt.show()

def plot_simulated_ground(theta, height):
    distances = []
    for i in range(-89, 89):
        # Calculate the x, y, and z components
        x = height * np.tan(np.deg2rad(theta))
        y = height
        z = height * np.tan(np.deg2rad(i))
        distance = np.sqrt(x*x + y*y + z*z)

        print(distance)
        distances.append(distance)
    r = np.arange(0, 2, 0.01)
    theta = 2 * np.pi * r

    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
    ax.plot(np.deg2rad([i + 90 for i in range(-89, 89)]), distances, '*')
    ax.set_rmax(2)
    ax.set_rlabel_position(-22.5)  # Move radial labels away from plotted line
    ax.grid(True)

    ax.set_title("A line plot on a polar axis", va='bottom')
    plt.show()

plot_simulated_ground(0, 1)
x_points = [-4, -3, -2, -1, 0, 1, 2, 3, 4]
print(x_points)
y_points = [2, 2, 2, 2, 2, 2, 2, 2, 2]
print(y_points)
print()
r_points = np.empty(9)
theta_points = np.empty(9)
phi_points = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]


for i in range(len(x_points)): 
    r, theta = cartesian_to_polar(x_points[i], y_points[i])
    r_points[i] = r
    theta_points[i] = theta

print("Computed r values")
print(r_points)
print()
print("Computed theta values")
print(theta_points)
print()

#plot_polar_points(r_points=r_points, theta_points=theta_points)



new_x_points = np.empty(9)
new_y_points = np.empty(9)
new_z_points = np.empty(9)

for j in range(len(x_points)):
    x, y, z = spherical_to_cartesian(r_points[j], theta_points[j], phi_points[j])
    new_x_points[j] = x
    new_y_points[j] = y
    new_z_points[j] = z


#plot_cartesian_points(x_points=new_x_points, y_points=new_y_points)

print("Recomputed x values")
print(new_x_points)
print()
print("Recomputed y values")
print(new_y_points)

r_points = np.empty(9)
theta_points = np.empty(9)


for i in range(len(x_points)): 
    r, theta, phi = cartesian_to_spherical(new_x_points[i], new_y_points[i], new_z_points[i])
    r_points[i] = r
    theta_points[i] = theta

fig, ax = plt.subplots(subplot_kw={'projection': '3d'})

ax.plot(theta_points, r_points)
ax.grid(True)
ax.set_xlabel("X axis")
ax.set_ylabel("Y axis")
ax.set_title("A line plot on a polar axis", va='bottom')

plt.show()

print("Computed r values")
print(r_points)
print()
print("Computed theta values")
print(theta_points)
print()