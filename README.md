# Laboratorio 4 Cinemática Directa Robot Phantom X ROS
Autores: Marcos Alfredo Fierro Sarria y Camilo Andres Apraez Mora
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

## Caracterización del robot y uso de la Toolbox:

Con ayuda de un calibrador se obtuvieron las medidas longitudinales de los eslabones del robot y con el programa dynamixel que ya se abordará más adelante podíamos ver los offset de los ángulos, en este caso home para conveniencia y así posteriormente usar las herramientas del toolbox, se simula el robot dentro del entorno de Matlab con Serial.Link y para la creación de la herramienta se usó .tool, para de esta manera asegurar la correcta orientación de la misma, el punto que tomamos fue en el centro del tcp, en este caso de la pinza. Todo ello, lo podemos, ver en el script ... 

que nos permita obtener las matrices de transformación homogénea:



Se plotean las distintas posiciones:

Mostramos las poses del robot:

![Home](https://github.com/MarcosAlferoFierroSarriamfierros/Laboratorio-4-Cinem-tica-Inversa-Robot-Phantom-Pitcher-X/assets/73545192/d6b3f6d7-2756-4c16-8d5f-d4991b6c1e8f)


![Pose2](https://github.com/MarcosAlferoFierroSarriamfierros/Laboratorio-4-Cinem-tica-Inversa-Robot-Phantom-Pitcher-X/assets/73545192/658b5c4c-70e8-41af-9ad1-0c8db249d758)


![Pose3](https://github.com/MarcosAlferoFierroSarriamfierros/Laboratorio-4-Cinem-tica-Inversa-Robot-Phantom-Pitcher-X/assets/73545192/a8e6495a-1bca-4c25-a35c-e4e93e2f8c0e)


![Pose4](https://github.com/MarcosAlferoFierroSarriamfierros/Laboratorio-4-Cinem-tica-Inversa-Robot-Phantom-Pitcher-X/assets/73545192/d7341b7e-ae26-4dfa-9d01-904615bbfc89)


![Pose5](https://github.com/MarcosAlferoFierroSarriamfierros/Laboratorio-4-Cinem-tica-Inversa-Robot-Phantom-Pitcher-X/assets/73545192/2da783d8-9884-4bf2-829d-919d914475a4)




## Análisis:
Con las medidas obtenidas en el inciso anterior, se procedió a obtener los parámetros DH que caraterizan al robot:

INSERTAR TABLA DH

## ROS:
Primero se debe asegurar de tener instalados todos los programas necesarios para hacer el entorno de Catkin y ROS, para así manipular los archivos necesarios para realizar práctica, en caso de que use una máquina virtual, puede que experimente fallas de conexión con el puerto serial del dispositivo, lo que impedirá acceder al robot, por ende, es de menester que el lector instale el Extension Pack correspondiente a la versión de la máquina virtual que haya instalado para tener acceso a todos los drivers (en este caso USB 3.0) para que reconozca el puerto USB0.
Una vez que se tengan los archivos de Catkin, se crean los scripts en las carpeta "src" dentro de los archivos de Dynamixel, el cual está contenido dentro de Catkin. Se debe lanzar el entorno ejecutando los comandos que se muestran:

![image](https://github.com/MarcosAlferoFierroSarriamfierros/Laboratorio-4-Cinem-tica-Inversa-Robot-Phantom-Pitcher-X/assets/73545192/f48d4eeb-767c-4e75-82c9-ce9f26d22e5e)

A continuación podemos ver, las carpetas ya mencionadas:

![image](https://github.com/MarcosAlferoFierroSarriamfierros/Laboratorio-4-Cinem-tica-Inversa-Robot-Phantom-Pitcher-X/assets/73545192/037ce11b-3fac-48d9-a0ac-68da6507c394)

![image](https://github.com/MarcosAlferoFierroSarriamfierros/Laboratorio-4-Cinem-tica-Inversa-Robot-Phantom-Pitcher-X/assets/73545192/81bdaf84-1c5f-49a1-817f-db6df11cc27c)

![image](https://github.com/MarcosAlferoFierroSarriamfierros/Laboratorio-4-Cinem-tica-Inversa-Robot-Phantom-Pitcher-X/assets/73545192/d6edac14-eb12-47ca-bca0-95579f792565)

![image](https://github.com/MarcosAlferoFierroSarriamfierros/Laboratorio-4-Cinem-tica-Inversa-Robot-Phantom-Pitcher-X/assets/73545192/9d382dbc-e5cd-40a7-956d-223d615ca2f4)



También se debe editar el archivo basic.yaml, el cual define etiquetas de identificación para los joints, este archivo podría ser cargado por un nodo que inicializa los motores de las articulacione según su ID, esto lo podemos ver en la carpeta:

![image](https://github.com/MarcosAlferoFierroSarriamfierros/Laboratorio-4-Cinem-tica-Inversa-Robot-Phantom-Pitcher-X/assets/73545192/b540ed24-be6e-44e3-9364-db425f563936)


El código presentado: Lab42v1.py, ofrece una serie de poses que el usuario puede seleccionar a través de la terminal y las cuales, el robot ejecuta

![Imagen de WhatsApp 2024-05-27 a las 19 52 47_a9ed55db](https://github.com/MarcosAlferoFierroSarriamfierros/Laboratorio-4-Cinem-tica-Inversa-Robot-Phantom-Pitcher-X/assets/73545192/bd171368-a33d-4823-a7b9-2b22dd07efab)

En general este código lo que hace es lo siguiente:

*Inicialización y Configuración: La función joint_publisher() inicia un nodo ROS, crea un publicador para el tópico /joint_trajectory y muestra un mensaje de bienvenida con las instrucciones.

*Input del Usuario: En un bucle, el programa espera que el usuario ingrese un comando. Según la tecla ingresada, se selecciona una posición predefinida para las articulaciones.

*Verificación de Límites: La función are_positions_within_limits(positions) verifica que las posiciones deseadas estén dentro de los límites permitidos.

*Publicación de la Trayectoria: Si las posiciones están dentro de los límites, se crea un mensaje JointTrajectory, se configura con las posiciones y se publica en el tópico /joint_trajectory.

*Suscripción para Estados de Articulaciones: Si el usuario ingresa '0', se llama a la función listener() que suscribe al nodo a los estados de las articulaciones y ejecuta la función de callback.

## Dynamixel

Usamos este software para verificar la conexión entre el computador y el robot, además de identificar el cero para establecer el Home del robot, como también los límites para cada motor, esto se podría extrapolar a cualquier tipo de robot, dada sus características:

INSERTAR FOTO DEL DYNAMIXEL



## Python + ROS + Toolbox
Finalmente, con el script realizado anteriormente, hacemos una interfaz gráfica del mismo, en el que se pueden ver las poses enviadas al robot. Se realizan con las poses sugeridas por el docente, sin embargo, la pose número 4 fue cambiada ligeramente, ya que interferia de forma negativa con el desenvolvimiento en el espacio del robot. Fue menester por ello, ajustar una serie de limitadores para cada articulación, para asegurar que la posición en cuestión no comprometiera el equilibrio del robot, el script en cuestión recibe el nombre de: functionalv8.py
![Imagen de WhatsApp 2024-05-27 a las 19 42 48_fe4aa501](https://github.com/MarcosAlferoFierroSarriamfierros/Laboratorio-4-Cinem-tica-Inversa-Robot-Phantom-Pitcher-X/assets/73545192/3bdf5e80-5559-49a1-abf3-bd391f94957f)


