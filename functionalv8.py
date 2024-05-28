import math
import os
import rospy
import tkinter as tk
from tkinter import PhotoImage
from PIL import Image, ImageTk
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

# Inicializa el nodo de ROS
rospy.init_node('robot_controller', anonymous=True)
pub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=10)

# Define los límites en radianes para cada articulación
joint_limits = {
    "joint_1": (-math.radians(130), math.radians(90)),
    "joint_2": (-math.radians(45), math.radians(45)),
    "joint_3": (-math.radians(155), math.radians(155)),
    "joint_4": (-math.radians(90), math.radians(90)),
    "tool": (-math.radians(90), math.radians(90))
}

# Variables para guardar las posiciones
previous_position = [0, 0, 0, 0, 0]
current_position = [0, 0, 0, 0, 0]

def check_position_limits(positions):
    joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "tool"]
    for i, position in enumerate(positions):
        joint_name = joint_names[i]
        lower_limit, upper_limit = joint_limits[joint_name]
        if not (lower_limit <= position <= upper_limit):
            return False, joint_name
    return True, None

def convert_radians_to_degrees(positions):
    return [round(math.degrees(p), 2) for p in positions]

def update_current_position_label(data):
    global current_position
    current_position = [round(p, 2) for p in data.position]
    degrees_position = convert_radians_to_degrees(current_position)
    real_position_label.config(text=f'Posición real (grados): \n {degrees_position}')

def send_joint_positions(positions, result_label):
    global previous_position
    previous_position = current_position[:]
    traj = JointTrajectory()
    traj.header.stamp = rospy.Time.now()
    traj.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "tool"]
    point = JointTrajectoryPoint()
    point.positions = positions
    point.time_from_start = rospy.Duration(1)
    traj.points.append(point)
    pub.publish(traj)
    degrees_position = convert_radians_to_degrees(positions)
    result_label.config(text=f'Posición enviada (grados): \n {degrees_position}')
    previous_degrees_position = convert_radians_to_degrees(previous_position)
    previous_position_label.config(text=f'Posición anterior (grados): \n {previous_degrees_position}')
    print('Posición enviada:', degrees_position)

def handle_position_send(positions, result_label):
    within_limits, joint_name = check_position_limits(positions)
    if within_limits:
        send_joint_positions(positions, result_label)
    else:
        result_label.config(text=f'Posición no válida para la articulación {joint_name}', font=("Helvetica", 10))

def load_and_resize_image(image_path, size=(300, 300)):
    img = Image.open(image_path)
    img = img.resize(size, Image.LANCZOS)
    return ImageTk.PhotoImage(img)

def build_gui():
    root = tk.Tk()
    root.title("Control de Robot")
    root.geometry("1000x800")
    root.configure(bg='blue')

    iconito = Image.open("/home/marcos_fierro/catkin_ws/src/dynamixel_one_motor/scripts/logoapp.png")
    resized_image = iconito.resize((30, 30))
    img = ImageTk.PhotoImage(resized_image)

    img_label = tk.Label(root, image=img)
    img_label.image = img
    img_label.place(x=10, y=10)

    label_actual = tk.Label(root, text="Pose actual:", bg='blue')
    label_actual.place(x=750, y=420)
    initial_image_path = "/home/marcos_fierro/catkin_ws/src/dynamixel_one_motor/scripts/poses/imágenes poses/Home.png"

    initial_img = load_and_resize_image(initial_image_path)
    label_actual = tk.Label(root, text="Pose anterior:", bg='blue')
    label_actual.place(x=150, y=420)
    img_label = tk.Label(root, image=initial_img)
    img_label.image = initial_img
    img_label.place(x=650, y=450)

    previous_image_label = load_and_resize_image("/home/marcos_fierro/catkin_ws/src/dynamixel_one_motor/scripts/poses/imágenes poses/Home.png")
    previous_image_label = tk.Label(root, image=previous_image_label)
    previous_image_label.image = previous_image_label
    previous_image_label.place(x=50, y=450)

    icono_chico = tk.PhotoImage(file="/home/marcos_fierro/catkin_ws/src/dynamixel_one_motor/scripts/logoapp.png")
    root.iconphoto(False, icono_chico, icono_chico)

    authors_label = tk.Label(root, text="Autores: Marcos Fierro y Camilo Apraez", font=("Helvetica", 12), bg='blue', fg='white')
    authors_label.pack(pady=10)

    images = {
        "1": load_and_resize_image("/home/marcos_fierro/catkin_ws/src/dynamixel_one_motor/scripts/poses/imágenes poses/Home.png"),
        "2": load_and_resize_image("/home/marcos_fierro/catkin_ws/src/dynamixel_one_motor/scripts/poses/imágenes poses/Pose2.png"),
        "3": load_and_resize_image("/home/marcos_fierro/catkin_ws/src/dynamixel_one_motor/scripts/poses/imágenes poses/Pose3.png"),
        "4": load_and_resize_image("/home/marcos_fierro/catkin_ws/src/dynamixel_one_motor/scripts/poses/imágenes poses/Pose4.png"),
        "5": load_and_resize_image("/home/marcos_fierro/catkin_ws/src/dynamixel_one_motor/scripts/poses/imágenes poses/Pose5.png"),
        "6": load_and_resize_image("/home/marcos_fierro/catkin_ws/src/dynamixel_one_motor/scripts/poses/imágenes poses/Pose5.png"),
        "9": load_and_resize_image("/home/marcos_fierro/catkin_ws/src/dynamixel_one_motor/scripts/poses/imágenes poses/Pose5.png")
    }

    previous_image_label = None
    previous_image = initial_img

    def button_click_handler(key):
        nonlocal previous_image_label, previous_image

        positions_dict = {
            '1': [round(math.radians(0), 2), round(math.radians(0), 2), round(math.radians(0), 2), round(math.radians(0), 2), round(math.radians(0), 2)],
            '2': [round(math.radians(45), 2), round(math.radians(25), 2), round(math.radians(20), 2), round(math.radians(-20), 2), round(math.radians(0), 2)],
            '3': [round(math.radians(-15), 2), round(math.radians(35), 2), round(math.radians(-30), 2), round(math.radians(30), 2), round(math.radians(0), 2)],
            '4': [round(math.radians(85), 2), round(math.radians(-10), 2), round(math.radians(55), 2), round(math.radians(25), 2), round(math.radians(0), 2)],
            '5': [round(math.radians(90), 2), round(math.radians(35), 2), round(math.radians(75), 2), round(math.radians(-45), 2), round(math.radians(0), 2)],
            '6': [round(math.radians(-365), 2), round(math.radians(200), 2), round(math.radians(590), 2), round(math.radians(700), 2), round(math.radians(1000), 2)],
            '9': [-0.3, 2.2, -2.5, -1.3, 0.3]
        }

        if key in positions_dict:
            handle_position_send(positions_dict[key], result_label)

            if previous_image_label:
                previous_image_label.place_forget()

            previous_image_label = tk.Label(root, image=previous_image)
            previous_image_label.image = previous_image
            previous_image_label.place(x=50, y=450)

            previous_image = images[key]

            img_label.config(image=images[key])
            img_label.image = images[key]

    button_texts = ["Posición 1: Home", "Posición 2", "Posición 3", "Posición 4", "Posición 5"]
    for i, text in enumerate(button_texts):
        button = tk.Button(root, text=text, command=lambda k=str(i+1): button_click_handler(k), bg='green', fg='white')
        button.pack(pady=5)
    
    button_guardado = tk.Button(root, text='Posición Guardada', command=lambda: button_click_handler('9'), width=25, bg='green', fg='white')
    button_guardado.pack(pady=5)

    global previous_position_label, result_label, real_position_label

    previous_position_label = tk.Label(root, text="Posición anterior:\n", font=("Helvetica", 12), bg='blue', fg='white')
    previous_position_label.pack(pady=10)
    
    result_label = tk.Label(root, text="Posición enviada:\n", font=("Helvetica", 12), bg='blue', fg='white')
    result_label.pack(pady=10)
    
    real_position_label = tk.Label(root, text="Posición real:\n", font=("Helvetica", 12), bg='blue', fg='white')
    real_position_label.pack(pady=10)
    
    rospy.Subscriber("/dynamixel_workbench/joint_states", JointState, update_current_position_label)

    root.mainloop()

if __name__ == '__main__':
    build_gui()
