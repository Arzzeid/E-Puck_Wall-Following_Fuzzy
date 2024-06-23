from controller import Robot, GPS
from flc import flc_compute
import csv

def run_robot(robot):
    timestep = 64
    error_sebelum = 0  
    kecepatan_max = 6.28 

    # Sensor
    sensor = []
    sensor_name = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
    for i in range(8):
        sensor.append(robot.getDevice(sensor_name[i]))
        sensor[i].enable(timestep)
    
    # Motor
    wheel = []
    wheel_name = ['left wheel motor', 'right wheel motor']
    for i in range(2):
        wheel.append(robot.getDevice(wheel_name[i]))
        wheel[i].setPosition(float('inf'))
        wheel[i].setVelocity(0.0)
    
    # GPS
    gps = robot.getDevice('gps')
    gps.enable(timestep)
    
    # Set point 
    set_point = 100
    
    # Membuat file CSV
    with open('robot_data.csv', mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['time', 'error', 'deltaError', 'fuzzy_control', 'sensor_0', 'sensor_1', 'sensor_2', 'sensor_3', 'motor_L', 'motor_R', 'pos_x', 'pos_y'])
        
        while robot.step(timestep) != -1:
            current_time = robot.getTime()  # Get current time
            # membaca nilai sensor
            sensor_val = [sensor[i].getValue() for i in range(8)]

            # menghitung error dan delta error
            nilai_error = set_point - sensor_val[2]
            nilai_delta_error = nilai_error - error_sebelum

            # flc
            fuzz_control = flc_compute(nilai_error, nilai_delta_error)
            
            # Update error sebelumnya
            error_sebelum = nilai_error
            
            # Wall-following logic
            if sensor_val[0] > 78:
                left = -kecepatan_max / 2
                right = kecepatan_max / 4
            elif nilai_error == 0:
                left = kecepatan_max
                right = kecepatan_max
            elif sensor_val[2] > 70 or sensor_val[1] > 70 or sensor_val[3] > 70:
                left = min(kecepatan_max + fuzz_control / 2, kecepatan_max)
                right = min(kecepatan_max - fuzz_control / 2, kecepatan_max)
            else:
                left = kecepatan_max / 4
                right = -kecepatan_max / 2

            # Set kecepatan motor
            wheel[0].setVelocity(left)
            wheel[1].setVelocity(right)

            # Get position
            pos = gps.getValues()
            pos_x, pos_y = pos[0], pos[1]

            # Tulis data ke CSV
            writer.writerow([
                f"{current_time:.2f}",
                f"{nilai_error:.2f}",
                f"{nilai_delta_error:.2f}",
                f"{fuzz_control:.2f}",
                f"{sensor_val[0]:.2f}",
                f"{sensor_val[1]:.2f}",
                f"{sensor_val[2]:.2f}",
                f"{sensor_val[3]:.2f}",
                f"{left:.2f}",
                f"{right:.2f}",
                f"{pos_x:.2f}",
                f"{pos_y:.2f}"
            ])

            # Print value
            print("error: {:.1f}".format(nilai_error),
                  "\tdeltaError: {:.1f}".format(nilai_delta_error), 
                  "\tfuzzyVal: {:.1f}".format(fuzz_control),
                  "\tsensor(0,1,2,3): ({:.1f}, {:.1f},  {:.1f},  {:.1f})"
                  .format(sensor_val[0], sensor_val[1], sensor_val[2], sensor_val[3]),
                  "\tmotor(L,R): {:.1f}, {:.1f}".format(left, right),
                  "\tpos: ({:.2f}, {:.2f})".format(pos_x, pos_y))

if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)
