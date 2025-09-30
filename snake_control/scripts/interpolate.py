import numpy as np
import csv

def interpolate_arrays(arrays, num_steps=10):
    result = []
    for i in range(len(arrays) - 1):
        start, end = np.array(arrays[i]), np.array(arrays[i+1])  # ensure numpy arrays
        for alpha in np.linspace(0, 1, num_steps, endpoint=False):
            interp = (1 - alpha) * start + alpha * end
            result.append(interp)
    result.append(np.array(arrays[-1]))  # last one
    return np.vstack(result)

num_points = []
knot_points = []
with open("/home/kallrax/biorobotics_lab/ros1_ws/src/snake_control/data/MappedCtrlr_0.050_0.350.csv", "r") as f:
    reader = csv.reader(f)
    for row in reader:
        q = [float(i) for i in row]
        knot_points.append(q)

for i in range(len(knot_points)-1):
    if i>76:
        num_points.append(10)
    else:
        num_points.append(10)

final = []
for i in range(len(num_points)):
    if i == 0 :
        final = interpolate_arrays(knot_points[i:i+2], num_points[i])
    else:
        final = np.concatenate((final,interpolate_arrays(knot_points[i:i+2], num_points[i])))

# order = [8, 4, 7, 14, 10, 3, 0, 1, 13, 12, 5, 15, 6, 2, 11, 9]
# final = final[:, order]

np.savetxt("interpolated_points.csv", final, delimiter=",", comments='', fmt="%.4f")

