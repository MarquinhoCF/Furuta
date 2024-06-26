from matplotlib import animation
import serial
import csv
import tkinter as tk
import re
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg 
from matplotlib.figure import Figure
import threading 
import time
import matplotlib.pyplot as plt
# Open the serial port
import serial.tools.list_ports

# Get the list of available COM ports
arduino_ports = [p.device for p in serial.tools.list_ports.comports()] #if 'Arduino' in p.description]


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


# Read data from the serial port
def read_and_process_data():
    if (start):
        # Read a line of data
        try:
            line = ser.readline().decode('utf-8', errors="strict").strip()
            # Split the line into values
            values = line.split(', ')
        except:
            ser.reset_input_buffer()
            
            
        # Check if the line has the correct number of values
        # Append the values to the lists
        try:
            if len(values) == 9:
                timestamps.append(float(values[0]))
                motor_counts.append(float(values[1]))
                motor_degrees.append(float(values[2]))
                motor_positions.append(float(values[3]))
                angular_velocities_motor.append(float(values[4]))
                pendulum_counts.append(float(values[5]))
                pendulum_degrees.append(float(values[6]))
                pendulum_positions.append(float(values[7]))
                angular_velocities_pendulum.append(float(values[8]))
        except:
            ser.reset_input_buffer()
            return

        if (print_serial_values):
            print("Invalid data:", values)
            print("Data length:", len(values))
            print("Data:", values)
            print("Line:", line)
            print("Timestamps:", timestamps)
            print("Motor Counts:", motor_counts)
            print("Motor Degrees:", motor_degrees)
            print("Motor Positions:", motor_positions)
            print("Angular Velocities Motor:", angular_velocities_motor)
            print("Pendulum Counts:", pendulum_counts)
            print("Pendulum Degrees:", pendulum_degrees)
            print("Pendulum Positions:", pendulum_positions)
            print("Angular Velocities Pendulum:", angular_velocities_pendulum)

        if(len(timestamps)):
            print(f"Timestamp: {timestamps[-1]}, Pulsos do Motor: {motor_counts[-1]}, Posição em graus do Motor: {motor_degrees[-1]}, Posição em rad do Motor: {motor_positions[-1]}, Velocidade Angular do Motor: {angular_velocities_motor[-1]}, Pulsos do Pêndulo: {pendulum_counts[-1]}, Posição em graus do Pêndulo: {pendulum_degrees[-1]}, Posição em rad do Pêndulo: {pendulum_positions[-1]}, Velocidade Angular do Pêndulo: {angular_velocities_pendulum[-1]}")



def update_plot(frame):
    
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
    
    ax.grid(True)
    plt.xlabel('Tempo (s)')
    plt.ylabel('Valor')
    plt.legend()


# Create the user interface
root = tk.Tk()

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

# Function to perform the swing-up action
def swing_up():
    # Implement the swing-up logic here
    ser.write(b'SwingUp\n')

def Equilibrar():
    ser.write(b'Equilibrar\n')

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


# Function to write the data as CSV
def write_data_as_csv():
    with open('data.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['Tempo', 'Pulsos Encoder Motor', 'Posição Angular do Motor [graus]', 'Posição Angular do Motor [rad]', 'Velocidade Angular do Motor [rad/s]', 'Pulsos Encoder Pêndulo', 'Posição Angular Pêndulo em [graus]', 'Posição Angular Pêndulo [rad]', 'Velocidade Angular do Pêndulo [rad/s]'])
        for timestamp, motor_count, motor_degree, motor_position, angular_velocity_motor, pendulum_count, pendulum_degree, pendulum_position, angular_velocity_pendulum in zip(timestamps, motor_counts, motor_degrees, motor_positions, angular_velocities_motor, pendulum_counts, pendulum_degrees, pendulum_positions, angular_velocities_pendulum):
            writer.writerow([timestamp, motor_count, motor_degree, motor_position, angular_velocity_motor, pendulum_count, pendulum_degree, pendulum_position, angular_velocity_pendulum])
    

# Function to close the window
def close_window():
    root.destroy()

# Create the toggle buttons
# Create the toggle buttons
toggle_motor_counts_button = tk.Button(root, text= "Esconder/Mostrar Pulsos Encoder Motor" , command=toggle_show_motor_counts)
toggle_motor_counts_button.pack(side=tk.TOP)
toggle_motor_positions_button = tk.Button(root, text="Esconder/Mostrar Posições em [rad] do Motor", command=toggle_show_motor_positions)
toggle_motor_positions_button.pack(side=tk.TOP)
toggle_motor_angular_velocities_button = tk.Button(root, text="Esconder/Mostrar Velocidade Angular do Motor [rad/s]", command=toggle_show_motor_angular_velocities)
toggle_motor_angular_velocities_button.pack(side=tk.TOP)
toggle_motor_degrees_button = tk.Button(root, text="Esconder/Mostrar Posições em [graus] do Motor", command=toggle_show_motor_degrees)
toggle_motor_degrees_button.pack(side=tk.TOP)
toggle_pendulum_counts_button = tk.Button(root, text="Esconder/Mostrar Pulsos do Pêndulo", command=toggle_show_pendulum_counts)
toggle_pendulum_counts_button.pack(side=tk.TOP)
toggle_pendulum_degrees_button = tk.Button(root, text="Esconder/Mostrar Posições em [graus] do Pendulo ", command=toggle_show_pendulum_degrees)
toggle_pendulum_degrees_button.pack(side=tk.TOP)
toggle_pendulum_positions_button = tk.Button(root, text="Esconder/Mostrar Posições em [rad] do Pendulo", command=toggle_show_pendulum_positions)
toggle_pendulum_positions_button.pack(side=tk.TOP)
toggle_pendulum_angular_velocities_button = tk.Button(root, text="Esconder/Mostrar Velocidade Angular do Pêndulo [rad/s]", command=toggle_show_pendulum_angular_velocities)
toggle_pendulum_angular_velocities_button.pack(side=tk.TOP)

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
ani = animation.FuncAnimation(fig, update_plot, interval=0.01, blit=False)


def main():
    while(True):
        print(arduino_ports)
        # Check if any Arduino devices are found
        if arduino_ports:
            # Open the serial port
            global ser
            ser = serial.Serial(arduino_ports[0], 112500)
            print("Arduino connected on", arduino_ports[0])
            break
        else:
            print("Arduino not found. Please check the connection.")
    
    # Start the main loop
    root.mainloop()

if __name__ == "__main__":
    main()