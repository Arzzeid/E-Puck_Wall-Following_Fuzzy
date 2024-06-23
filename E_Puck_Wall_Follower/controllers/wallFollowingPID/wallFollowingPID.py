from controller import Robot, GPS
import csv

def calculate_motor(signal):
    return (signal / 100) * 6.28

def run_robot(robot):
    timestep = 64

    # define sensor
    sensor = []
    sensor_name = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
    for i in range(8):
        sensor.append(robot.getDevice(sensor_name[i]))
        sensor[i].enable(timestep)

    # define motor
    wheel = []
    wheel_name = ['left wheel motor', 'right wheel motor']
    for i in range(2):
        wheel.append(robot.getDevice(wheel_name[i]))
        wheel[i].setPosition(float('inf'))
        wheel[i].setVelocity(0.0)

    # define GPS
    gps = robot.getDevice('gps')
    gps.enable(timestep)

    # parameter
    pid_parameter = [0.55, 0.0004, 1]  # Kp Ki Kd
    error = [0, 0, 0]
    set_point = 100
    control = [0, 0, 0]
    pid_control = 0
    sensor_val = [0, 0, 0, 0, 0, 0, 0, 0]

    # Prepare for data logging
    log_data = []

    while robot.step(timestep) != -1:
        current_time = robot.getTime()

        normal_speed = 80
        fast_speed = 100

        for i in range(8):
            sensor_val[i] = sensor[i].getValue()

        # kendali proporsional
        error[0] = set_point - sensor_val[2]
        control[0] = error[0] * pid_parameter[0]

        # kendali integral
        error[1] = error[1] + error[0]
        if error[1] > 150:
            error[1] = 150
        if error[1] <= -150:
            error[1] = -150
        control[1] = error[1] * pid_parameter[1]

        # kendali differensial
        control[2] = (error[0] - error[2]) * pid_parameter[2]
        error[2] = error[0]

        pid_control = control[0] + control[1] + control[2]

        if pid_control >= (fast_speed - normal_speed - 1):
            pid_control = (fast_speed - normal_speed - 1)
        if pid_control <= -(fast_speed - normal_speed - 1):
            pid_control = -(fast_speed - normal_speed - 1)

        max_speed = calculate_motor(fast_speed)
        # cek dinding depan
        if sensor_val[0] > 80:
            left = -max_speed
            right = max_speed
        else:
            if error[0] >= -5 and error[0] <= 5:
                left = max_speed
                right = max_speed
            else:
                if sensor_val[2] > 80:
                    speed = calculate_motor(normal_speed)
                    pid_control = calculate_motor(pid_control)
                    left = speed + pid_control
                    right = speed - pid_control
                else:
                    left = max_speed
                    right = max_speed / 6

        wheel[0].setVelocity(left)
        wheel[1].setVelocity(right)

        # Get position
        pos = gps.getValues()
        pos_x, pos_y = pos[0], pos[1]

        # Log data with 2 decimal places
        log_data.append([
            f"{current_time:.2f}",  # Format time to 2 decimal places
            f"{error[0]:.2f}",      # Format error to 2 decimal places
            f"{left:.2f}",          # Format left motor speed to 2 decimal places
            f"{right:.2f}",         # Format right motor speed to 2 decimal places
            f"{pos_x:.2f}",         # Format x position to 2 decimal places
            f"{pos_y:.2f}"          # Format y position to 2 decimal places
        ])
        print(f"Sensor[2]: {sensor_val[2]:.2f}, Pos: ({pos_x:.2f}, {pos_y:.2f})")

    # Write data to CSV file
    with open('robot_data_pid.csv', 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['time', 'error', 'motor_L', 'motor_R', 'pos_x', 'pos_y'])
        writer.writerows(log_data)

if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)
