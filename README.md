# Laboratorio 4 Cinemática Directa Robot Phantom X ROS

## Autores: Marcos Alfredo Fierro Sarria y Camilo Andres Apraez Zamora

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

Con ayuda de un calibrador se obtuvieron las medidas longitudinales de los eslabones del robot y con el programa dynamixel que ya se abordará más adelante podíamos ver los offset de los ángulos, en este caso home para conveniencia y así posteriormente usar las herramientas del toolbox, se simula el robot dentro del entorno de Matlab con Serial.Link y para la creación de la herramienta se usó .tool, para de esta manera asegurar la correcta orientación de la misma, el punto que tomamos fue en el centro del tcp, en este caso de la pinza. Todo ello, lo podemos, ver en el script https://github.com/MarcosAlferoFierroSarriamfierros/Laboratorio-4-Cinem-tica-Inversa-Robot-Phantom-Pitcher-X/blob/main/Laboratorio_cinematica_directa.mlx.

A continuación, podemos ver la tabla DH obtenida:

![image](https://github.com/MarcosAlferoFierroSarriamfierros/Laboratorio-4-Cinem-tica-Inversa-Robot-Phantom-Pitcher-X/assets/83042758/4ac89c02-768a-4f35-810f-db7aa3fffc91)

a partir de ello, medio una función creada por nosotros:

function T = calcular_DH(theta, d, a, alpha)
    % Convertir ángulos de grados a radianes si es necesario
    if ischar(theta)
        theta_sym = str2sym(theta); % Convertir la cadena a un símbolo
        theta = deg2rad(theta_sym); % Convertir de grados a radianes
    else
        theta_sym = theta; % Mantener theta como un símbolo
        theta = deg2rad(theta); % Convertir de grados a radianes
    end

    % Convertir distancias a números simbólicos si es necesario
    if ischar(d)
        d = str2sym(d);
    end

    % Convertir distancias a números simbólicos si es necesario
    if ischar(a)
        a = str2sym(a);
    end
    
    % Convertir alfas de grados a radianes si es necesario
    if ischar(alpha)
        alpha = str2sym(alpha); % Convertir la cadena a un símbolo
        alpha = deg2rad(alpha); % Convertir de grados a radianes
    else
        alpha = deg2rad(alpha); % Convertir de grados a radianes
    end

    % Calcular la matriz de transformación de Denavit-Hartenberg
    T = [cos(theta_sym), -sin(theta_sym)*cos(alpha), sin(theta_sym)*sin(alpha), a*cos(theta_sym);
         sin(theta_sym), cos(theta_sym)*cos(alpha), -cos(theta_sym)*sin(alpha), a*sin(theta_sym);
         0, sin(alpha), cos(alpha), d;
         0, 0, 0, 1];

    % Expresar la matriz de transformación simbólicamente si es necesario
    if isa(theta_sym, 'sym') || isa(d, 'sym') || isa(a, 'sym') || isa(alpha, 'sym')
        T = simplify(T); % Simplificar la expresión simbólica
    end
end

donde introducimos los parámetros obtenidos de la tabla DH, para así hallar las MTH de cada eslabón:

*MTH1*

![image](https://github.com/MarcosAlferoFierroSarriamfierros/Laboratorio-4-Cinem-tica-Inversa-Robot-Phantom-Pitcher-X/assets/83042758/637a8270-b518-423e-91ca-fbfd83638ad2)

*MTH2*

![image](https://github.com/MarcosAlferoFierroSarriamfierros/Laboratorio-4-Cinem-tica-Inversa-Robot-Phantom-Pitcher-X/assets/83042758/77f4acc3-1aed-481c-9a51-0fa23c0bb1f1)

*MTH3*

![image](https://github.com/MarcosAlferoFierroSarriamfierros/Laboratorio-4-Cinem-tica-Inversa-Robot-Phantom-Pitcher-X/assets/83042758/b381111a-44f4-41d9-afed-ed642a241102)

*MTH4*

![image](https://github.com/MarcosAlferoFierroSarriamfierros/Laboratorio-4-Cinem-tica-Inversa-Robot-Phantom-Pitcher-X/assets/83042758/842aa97e-6dad-4996-8733-c728d1c094a0)

*MTH4_tool*

![image](https://github.com/MarcosAlferoFierroSarriamfierros/Laboratorio-4-Cinem-tica-Inversa-Robot-Phantom-Pitcher-X/assets/83042758/9987d431-d724-4051-aad1-1ef319bf067e)

En esta MTH, solo vemos una traslación con respecto al eslabón 4, la cual es la distancia en "Z", del joint 4 hasta la mitad de la pinza, que es lo que habíamos mencionado anteriormente.

*MTH_base_tcp*

A continuación encontramos la MTH, representada desde la base hasta el tcp, es decir la cinemática directa.

![image](https://github.com/MarcosAlferoFierroSarriamfierros/Laboratorio-4-Cinem-tica-Inversa-Robot-Phantom-Pitcher-X/assets/83042758/8b5ff03c-8a52-4bd7-acf0-780aa29837e5)


Se plotean las distintas posiciones y de una vez realizamos la comparación con las imágenes reales del manipulador PhantomX Pincher, de acuerdo a las poses que sugeríamos desde la interfaz:

# Gráficas digitales vs Imágenes reales








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

*Inicialización y Configuración:* La función joint_publisher() inicia un nodo ROS, crea un publicador para el tópico /joint_trajectory y muestra un mensaje de bienvenida con las instrucciones.

*Input del Usuario:* En un bucle, el programa espera que el usuario ingrese un comando. Según la tecla ingresada, se selecciona una posición predefinida para las articulaciones.

*Verificación de Límites:* La función are_positions_within_limits(positions) verifica que las posiciones deseadas estén dentro de los límites permitidos.

*Publicación de la Trayectoria:* Si las posiciones están dentro de los límites, se crea un mensaje JointTrajectory, se configura con las posiciones y se publica en el tópico /joint_trajectory.

*Suscripción para Estados de Articulaciones:* Si el usuario ingresa '0', se llama a la función listener() que suscribe al nodo a los estados de las articulaciones y ejecuta la función de callback.

## Dynamixel

Usamos este software para verificar la conexión entre el computador y el robot, además de identificar el cero para establecer el Home del robot, como también los límites para cada motor, esto se podría extrapolar a cualquier tipo de robot, dada sus características:

INSERTAR FOTO DEL DYNAMIXEL



## Python + ROS + Toolbox
Finalmente, con el script realizado anteriormente, hacemos una interfaz gráfica del mismo, en el que se pueden ver las poses enviadas al robot. Se realizan con las poses sugeridas por el docente, sin embargo, la pose número 4 fue cambiada ligeramente, ya que interferia de forma negativa con el desenvolvimiento en el espacio del robot. Fue menester por ello, ajustar una serie de limitadores para cada articulación, para asegurar que la posición en cuestión no comprometiera el equilibrio del robot, el script en cuestión recibe el nombre de: functionalv8.py
![Imagen de WhatsApp 2024-05-27 a las 19 42 48_fe4aa501](https://github.com/MarcosAlferoFierroSarriamfierros/Laboratorio-4-Cinem-tica-Inversa-Robot-Phantom-Pitcher-X/assets/73545192/3bdf5e80-5559-49a1-abf3-bd391f94957f)

## Logo

Definimos dos logos, uno con nuestras iniciales con un robot y otro con un robot que tiene las iniciales de la UNAL, en la interfaz tuvimos este incoveniente de ponerlo, por cuestión de tiempo no se pudo solucionar pero se espera poder colocarlo, pero se presentan a continuación:




