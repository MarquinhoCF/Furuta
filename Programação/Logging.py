from matplotlib import animation
import serial
import csv
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg 
import time
import matplotlib.pyplot as plt
# Open the serial port
import serial.tools.list_ports

# Get the list of available COM ports
arduino_ports = [p.device for p in serial.tools.list_ports.comports()] #if 'Arduino' in p.description]

print(arduino_ports)
# Check if any Arduino devices are found
if arduino_ports:
    # Open the serial port
    ser = serial.Serial(arduino_ports[0], 9600)
    print("Arduino connected on", arduino_ports[0])
else:
    print("Arduino not found. Please check the connection.")
    time.sleep(1)


# Create lists to store the data
timestamps = [0]
motor_counts = [0]
motor_positions = [0]
angular_velocities_motor = [0]
motor_degrees = [0]
pendulum_counts = [0]
pendulum_positions = [0]
pendulum_degrees = [0]
angular_velocities_pendulum = [0]
pwm = [0]
volts = [0]

start = False
print_serial_values = False

show_motor_positions = True
show_motor_counts = True
show_motor_angular_velocities = True
show_motor_degrees = True
show_pendulum_counts = True
show_pendulum_positions = True
show_pendulum_degrees = True
show_pendulum_angular_velocities = True
show_pwm = True
show_volts = True

# Read data from the serial port
def read_and_process_data():
    # Read a line of data
    if(start):
        line = ser.readline().decode('utf-8', errors="strict").strip()
        # Split the line into values
        values = line.split(', ')   

    timestamps.append(float(values[0]))
    motor_counts.append(float(values[1]))
    motor_degrees.append(float(values[2]))
    motor_positions.append(float(values[3]))
    angular_velocities_motor.append(float(values[4]))
    pendulum_counts.append(float(values[5]))
    pendulum_degrees.append(float(values[6]))
    pendulum_positions.append(float(values[7]))
    angular_velocities_pendulum.append(float(values[8]))
    pwm.append(float(values[9]))
    volts.append(float(values[10]))


def update_plot(_):
    if(start):
        read_and_process_data()
    
    plt.cla()

    if show_motor_positions and start:
        plt.plot(timestamps, motor_positions, label='Posição Angular do Motor [rad]')
    if  show_motor_positions == False:
        plt.plot([], [], label='Posição Angular do Motor [rad]')

    if show_motor_angular_velocities and start:
        plt.plot(timestamps, angular_velocities_motor, label='Velocidade Angular do Motor [rad/s]')
    if show_motor_angular_velocities == False:
        plt.plot([], [], label='Velocidade Angular do Motor [rad/s]')

    if show_motor_degrees and start:
        plt.plot(timestamps, motor_degrees, label='Posicao do Motor [graus]')
    if show_motor_degrees == False:
        plt.plot([], [], label='Posicao do Motor [graus]')

    if show_pendulum_counts and start:
        plt.plot(timestamps, pendulum_counts, label='Pulsos Encoder Pêndulo')
    if show_pendulum_counts == False:
        plt.plot([], [], label='Pulsos Encoder Pêndulo')

    if show_pendulum_positions and start:
        plt.plot(timestamps, pendulum_positions, label='Posição Pêndulo em [rad]')
    if show_pendulum_positions == False:
        plt.plot([], [], label='Posição Pêndulo em [rad]')

    if show_pendulum_degrees and start:
        plt.plot(timestamps, pendulum_degrees, label='Posição do Pêndulo [graus]')
    if show_pendulum_degrees == False:
        plt.plot([], [], label='Posição do Pêndulo [graus]')

    if show_pendulum_angular_velocities and start:
        plt.plot(timestamps, angular_velocities_pendulum, label='Velocidade Angular do Pêndulo [rad/s]')
    if show_pendulum_angular_velocities == False:
        plt.plot([], [], label='Velocidade Angular do Pêndulo [rad/s]')

    if show_pwm and start:
        plt.plot(timestamps, pwm, label='PWM')
    if show_pwm == False:
        plt.plot([], [], label='PWM')

    if show_volts and start:
        plt.plot(timestamps, volts, label='Tensao')
    if show_volts == False:
        plt.plot([], [], label='Tensao')
    
    ax.grid(True)
    plt.xlabel('Tempo (s)')
    plt.ylabel('Valor')

# Function to toggle the value of show_motor_positions
def toggle_show_motor_counts():
    global show_motor_counts
    show_motor_counts = not show_motor_counts

# Function to toggle the value of show_motor_angular_positions
def toggle_show_motor_positions():
    global show_motor_positions
    show_motor_positions = not show_motor_positions

# Function to toggle the value of show_motor_angular_velocities
def toggle_show_motor_angular_velocities():
    global show_motor_angular_velocities
    show_motor_angular_velocities = not show_motor_angular_velocities

# Function to toggle the value of show_motor_velocities
def toggle_show_motor_degrees():
    global show_motor_degrees
    show_motor_degrees = not show_motor_degrees

# Function to toggle the value of show_pendulum_positions
def toggle_show_pendulum_counts():
    global show_pendulum_counts
    show_pendulum_counts = not show_pendulum_counts

# Function to toggle the value of show_pendulum_velocities
def toggle_show_pendulum_positions():
    global show_pendulum_positions
    show_pendulum_positions = not show_pendulum_positions

# Function to toggle the value of show_pendulum_angular_positions
def toggle_show_pendulum_degrees():
    global show_pendulum_degrees
    show_pendulum_degrees = not show_pendulum_degrees

# Function to toggle the value of show_pendulum_angular_velocities
def toggle_show_pendulum_angular_velocities():
    global show_pendulum_angular_velocities
    show_pendulum_angular_velocities = not show_pendulum_angular_velocities

# Function to toggle the value of show_pwm
def toggle_show_pwm():
    global show_pwm
    show_pwm = not show_pwm

# Function to perform the swing-up action
def swing_up():
    # Implement the swing-up logic here
    ser.write(b'SwingUp\n')

def Equilibrar():
    ser.write(b'Equilibrar\n')


root = tk.Tk()
root.title("Data Logger")

#Create the Equilibrar button
equilibrar_button = tk.Button(root, text="Equilibrar", command=Equilibrar)
equilibrar_button.pack(side=tk.BOTTOM)


# Create the SwingUp button
swing_up_button = tk.Button(root, text="SwingUp", command=swing_up)
swing_up_button.pack(side=tk.BOTTOM)

# Function to send the start command on serial
def send_start_command():
    ser.write(b'Start\n')
    global start
    start = True

# Function to send the stop command on serial
def send_stop_command():
    ser.write(b'Stop\n')
    global start
    start = False
    ser.reset_input_buffer()
    reset_data()
    

# Function to send the reset command on serial
def reset():
    ser.write(b'Reset\n')

# Function to reset the data
def reset_data():
    ser.reset_input_buffer()
    # Send reset command on the serial
    reset()
    # Clear the plot
    plt.cla()
    
    # Clear the original lists
    timestamps.clear()
    motor_positions.clear()
    motor_counts.clear()
    motor_degrees.clear()
    angular_velocities_motor.clear();
    pendulum_positions.clear()
    pendulum_degrees.clear()
    pendulum_counts.clear()
    angular_velocities_pendulum.clear()
    pwm.clear()
    volts.clear()


# Function to write the data as CSV
def write_data_as_csv():
    global pwm
    global volts
    pwm = [0 if x == 0 else x for x in pwm]
    volts = [0 if x == 0 else x for x in volts]

    with open('data.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['Tempo', 'Pulsos Encoder Motor', 'Posição Angular do Motor [graus]', 'Posição Angular do Motor [rad]', 'Velocidade Angular do Motor [rad/s]', 'Pulsos Encoder Pêndulo', 'Posição Angular Pêndulo em [graus]', 'Posição Angular Pêndulo [rad]', 'Velocidade Angular do Pêndulo [rad/s]', 'PWM', 'Tensao'])
        for timestamp, motor_count, motor_degree, motor_position, angular_velocity_motor, pendulum_count, pendulum_degree, pendulum_position, angular_velocity_pendulum, pwm, volts  in zip(timestamps, motor_counts, motor_degrees, motor_positions, angular_velocities_motor, pendulum_counts, pendulum_degrees, pendulum_positions, angular_velocities_pendulum, pwm, volts):
            writer.writerow([timestamp, motor_count, motor_degree, motor_position, angular_velocity_motor, pendulum_count, pendulum_degree, pendulum_position, angular_velocity_pendulum, pwm, volts])
    

# Function to close the window
def close_window():
    root.destroy()

# Create the toggle buttons
# Create the toggle buttons
# toggle_motor_counts_button = tk.Button(root, text= "Esconder/Mostrar Pulsos Encoder Motor" , command=toggle_show_motor_counts)
# toggle_motor_counts_button.pack(side=tk.TOP)
# toggle_motor_positions_button = tk.Button(root, text="Esconder/Mostrar Posições em [rad] do Motor", command=toggle_show_motor_positions)
# toggle_motor_positions_button.pack(side=tk.TOP)
# toggle_motor_angular_velocities_button = tk.Button(root, text="Esconder/Mostrar Velocidade Angular do Motor [rad/s]", command=toggle_show_motor_angular_velocities)
# toggle_motor_angular_velocities_button.pack(side=tk.TOP)
# toggle_motor_degrees_button = tk.Button(root, text="Esconder/Mostrar Posições em [graus] do Motor", command=toggle_show_motor_degrees)
# toggle_motor_degrees_button.pack(side=tk.TOP)
# toggle_pendulum_counts_button = tk.Button(root, text="Esconder/Mostrar Pulsos do Pêndulo", command=toggle_show_pendulum_counts)
# toggle_pendulum_counts_button.pack(side=tk.TOP)
# toggle_pendulum_degrees_button = tk.Button(root, text="Esconder/Mostrar Posições em [graus] do Pendulo ", command=toggle_show_pendulum_degrees)
# toggle_pendulum_degrees_button.pack(side=tk.TOP)
# toggle_pendulum_positions_button = tk.Button(root, text="Esconder/Mostrar Posições em [rad] do Pendulo", command=toggle_show_pendulum_positions)
# toggle_pendulum_positions_button.pack(side=tk.TOP)
# toggle_pendulum_angular_velocities_button = tk.Button(root, text="Esconder/Mostrar Velocidade Angular do Pêndulo [rad/s]", command=toggle_show_pendulum_angular_velocities)
# toggle_pendulum_angular_velocities_button.pack(side=tk.TOP)
# toggle_pwm_button = tk.Button(root, text="Esconder/Mostrar PWM", command=toggle_show_pwm)
# toggle_pwm_button.pack(side=tk.TOP)
# toggle_volts_button = tk.Button(root, text="Esconder/Mostrar Tensão", command=toggle_show_volts)
# toggle_volts_button.pack(side=tk.TOP)

#Create the reset button
reset_button = tk.Button(root, text="Reset", command=reset_data)
reset_button.pack(side=tk.BOTTOM)

# Create the write button
write_button = tk.Button(root, text="Write CSV", command=write_data_as_csv)
write_button.pack(side=tk.BOTTOM)
# Create the start button
start_button = tk.Button(root, text="Start", command=send_start_command)
start_button.pack(side=tk.BOTTOM)
# Create the stop button
stop_button = tk.Button(root, text="Stop", command=send_stop_command)
stop_button.pack(side=tk.BOTTOM)

#Create the reset button
reset_data_button = tk.Button(root, text="Reset_data", command=reset_data)
reset_data_button.pack(side=tk.BOTTOM)


# Create the plot
fig, ax = plt.subplots(figsize=(8, 6))
fig.canvas.manager.set_window_title('Data Logger')
canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

# Create the animation

ani = animation.FuncAnimation(fig, update_plot, frames=len(timestamps), interval=1, cache_frame_data=True)


# Start the main loop
root.mainloop()
