import matplotlib.pyplot as plt
import csv


with open("/home/charles/udem-fall19-public/catkin_ws/src/charles-ros-core/logs/carCmdData.txt", mode = 'r') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    line_count = 0
    for row in csv_reader:
        if line_count == 1:
            t0 = float(row[0])/(10 ** 9)
            t = [0]
            v = [float(row[4])]
            omega = [float(row[5])]
        elif line_count > 1:
            t.append(float(row[0])/(10 ** 9) - t0)
            v.append(float(row[4]))
            omega.append(float(row[5]))
            if (float(row[0])/(10**9) - t0) > 30:
                break
        line_count += 1
        
plt.figure(1)
plt.plot(t,omega)
plt.xlabel("Time (s)")
plt.ylabel("Commanded Omega (rad/s)")
plt.title("Commanded Angular Velocity")
plt.show()

plt.figure(2)
plt.plot(t,v)
plt.xlabel("Time (s)")
plt.ylabel("Commanded v (rad/s)")
plt.title("Commanded Forward Velocity")
plt.show()
print(f'Processed {line_count} lines.')


with open("/home/charles/udem-fall19-public/catkin_ws/src/charles-ros-core/logs/lanePoseData.txt", mode = 'r') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    line_count = 0
    for row in csv_reader:
        if line_count == 1:
            t0 = float(row[0])/(10 ** 9)
            t = [0]
            d = [float(row[4])]
            theta = [float(row[7])]
        elif line_count > 1:
            t.append(float(row[0])/(10 ** 9) - t0)
            d.append(float(row[4]))
            theta.append(float(row[7]))
            if (float(row[0])/(10**9) - t0) > 30:
                break
        line_count += 1
        
plt.figure(3)
plt.plot(t,d)
plt.xlabel("Time (s)")
plt.ylabel("d (m)")
plt.title("Cross-track error")
plt.show()

plt.figure(4)
plt.plot(t,theta)
plt.xlabel("Time (s)")
plt.ylabel("phi (rad)")
plt.title("Heading Error")
plt.show()
print(f'Processed {line_count} lines.')