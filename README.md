# Laboratorio 4 Cinemática Directa Robot Phantom X ROS
Autores: Marcos Alfredo Fierro Sarria y Andrés Camilo Apraez Mora
En el siguiente informe se muestra el proceso realizado para llevar a cabo la actividad propuesta.
## Requisitos:
Ubuntu versión 20.xx preferible 20.04 LTS con ROS.
Espacio de trabajo para catkin correctamente configurado.
Paquetes de Dynamixel Workbench. https://github.com/fegonzalez7/rob_unal_clase3
Paquete del robot Phantom X: https://github.com/felipeg17/px_robot.
Python o MATLAB 2015b o superior instalado en el equipo.
Robotics toolbox de Mathworks (Disponible desde la versión 2015 en adelante).
Toolbox de robótica de Peter Corke.
Un (1) manipulador Phantom X Pincher
## Caracterización del robot:
Con ayuda de un calibrador se obtuvieron las medidas del robot, las cuales puede ver en la imagen:

INSERTAR MEDIDAS

## Análisis:
Con las medidas obtenidas en el inciso anterior, se ´procedió a obtener los parámetros DH que caraterizan al robot:

INSERTAR TABLA DH

## ROS:
Primero se debe asegurar de tener instalados todos los programas necesarios para hacer el entorno de Catkin y ROS, en caso de que use una máquina virtual, puede que experimente fallas de conexión con el puerto serial del dispositivo, lo que impedirá acceder al robot, por ende, es de menester que el lector instale el Extension Pack correspondiente a la versión de la máquina virtual que haya instalado para tener acceso a todos los drivers que requiere la máquina.
Una vez que se tengan los archivos de Catkin, se crean los scripts en las carpeta "src" dentro de los archivos de Dynamixel, el cual está contenido dentro de Catkin. Se debe lanzar el entorno ejecutando los comandos que se muestran:

INSERTAR INITIAL CONFIG OF CATKIN

El código presentado ofrece una serie de poses que el usuario puede seleccionar através de la terminal y las cuales, el robot ejecuta:

INSERTAR IMAGEN CONSOLA EJECUTÁNDOSE


## Toolbox
Usando las herramientas del toolbox, se simula el robot dentro del entorno de Matlab. Con los parámetros DH obtenidos en secciones anteriores, se usó el comando SerialLink para crear el robot. Se hace una función que nos permita obtener las matrices de transformación homogénea:

INSERTAR FOTO DE LA FUNCIÓN MTH

Se plotean las distintas posiciones:

INSERTAR POSICIONES

## Python + ROS + Toolbox
Finalmente, con el script realizado anteriormente, hacemos una interfaz gráfica del mismo, en el que se pueden ver las poses enviadas al robot. Se realizan con las poses sugeridas por el docente, sin embargo, la pose número 4 fue cambiada ligeramente, ya que interferia de forma negativa con el desnvolvimiento en el espacio del robot. Fue menester por ello, ajustar una serie de limitadores para cada articulación, para asegurar que la posición en cuestión no comprometiera el equilibrio del robot.
