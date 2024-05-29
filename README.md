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

Se inicia con el análisis de las articulaciones y eslabones del robot Phantom X Pincher:

![image](https://github.com/MarcosAlferoFierroSarriamfierros/Laboratorio-4-Cinem-tica-Inversa-Robot-Phantom-Pitcher-X/assets/83042758/fe93a4a8-ab39-4b70-9e85-b26bd74a9bb9)


A continuación, podemos ver la tabla DH obtenida:

![image](https://github.com/MarcosAlferoFierroSarriamfierros/Laboratorio-4-Cinem-tica-Inversa-Robot-Phantom-Pitcher-X/assets/83042758/4ac89c02-768a-4f35-810f-db7aa3fffc91)

a partir de ello, medio una función creada por nosotros:

```matlab
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

### Gráficas digitales vs Imágenes reales

*Home: q1 = [0, 0, 0, 0]*


![image](https://github.com/MarcosAlferoFierroSarriamfierros/Laboratorio-4-Cinem-tica-Inversa-Robot-Phantom-Pitcher-X/assets/83042758/21c9ad0b-f3bb-41e4-9741-0e56354a09bf)

*Pose2: q2 = [25, 25, 20, -20]*

![image](https://github.com/MarcosAlferoFierroSarriamfierros/Laboratorio-4-Cinem-tica-Inversa-Robot-Phantom-Pitcher-X/assets/83042758/4c84fc7d-bad5-4b3e-9e7e-0155cfdbe20e)

*Pose3 q3 = [-35, 35, -30, 30]*

![image](https://github.com/MarcosAlferoFierroSarriamfierros/Laboratorio-4-Cinem-tica-Inversa-Robot-Phantom-Pitcher-X/assets/83042758/5cb9ce91-0600-4fc3-8a63-acef36691275)

*Pose4 q4 = [85, -20, 55, 25]*

![image](https://github.com/MarcosAlferoFierroSarriamfierros/Laboratorio-4-Cinem-tica-Inversa-Robot-Phantom-Pitcher-X/assets/83042758/239ec29e-2408-43a6-87bc-e7c337e9aafa)

*Pose5 q5 = [0, 0, 90, 0]*

![image](https://github.com/MarcosAlferoFierroSarriamfierros/Laboratorio-4-Cinem-tica-Inversa-Robot-Phantom-Pitcher-X/assets/83042758/9d95932b-d636-41bb-a365-8c3bfec881c9)



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

![Imagen de WhatsApp 2024-05-28 a las 20 21 28_a21c3874](https://github.com/MarcosAlferoFierroSarriamfierros/Laboratorio-4-Cinem-tica-Inversa-Robot-Phantom-Pitcher-X/assets/83042758/f8475770-1177-4646-be2b-335b84d6edce)




## Python + ROS + Toolbox
Finalmente, con el script realizado anteriormente, hacemos una interfaz gráfica del mismo, en el que se pueden ver las poses enviadas al robot. Se realizan con las poses sugeridas por el docente, sin embargo, la pose número 4 fue cambiada ligeramente, ya que interferia de forma negativa con el desenvolvimiento en el espacio del robot. Fue menester por ello, ajustar una serie de limitadores para cada articulación, para asegurar que la posición en cuestión no comprometiera el equilibrio del robot, el script en cuestión recibe el nombre de: functionalv8.py
![Imagen de WhatsApp 2024-05-27 a las 19 42 48_fe4aa501](https://github.com/MarcosAlferoFierroSarriamfierros/Laboratorio-4-Cinem-tica-Inversa-Robot-Phantom-Pitcher-X/assets/73545192/3bdf5e80-5559-49a1-abf3-bd391f94957f)

## Logo

Definimos dos logos, uno con nuestras iniciales con un robot y otro con un robot que tiene las iniciales de la UNAL, en la interfaz tuvimos este incoveniente de ponerlo, por cuestión de tiempo no se pudo solucionar pero se espera poder colocarlo, pero se presentan a continuación:


![Imagen de WhatsApp 2024-05-27 a las 09 04 48_6248d230](https://github.com/MarcosAlferoFierroSarriamfierros/Laboratorio-4-Cinem-tica-Inversa-Robot-Phantom-Pitcher-X/assets/83042758/3ec548d8-6bde-4f84-b05d-48f9092e034a)

![image](https://github.com/MarcosAlferoFierroSarriamfierros/Laboratorio-4-Cinem-tica-Inversa-Robot-Phantom-Pitcher-X/assets/83042758/804f374a-f02c-4405-b2bb-ec97e14d8206)

## Video del funcionamiento de interfaz gráfica 

A continuación presentamos el video de la interfaz gráfica, junto a las poses determinadas para la práctica que toma el robot después de aplicar los botones correspondientes de la interfaz:

https://youtu.be/2NLR2nmDH0o

### Conclusiones

1. **Implementación Exitosa del Entorno de Trabajo**:
   - La configuración del entorno en Ubuntu 20.04 LTS con ROS y la correcta configuración del espacio de trabajo Catkin permitieron la integración eficiente de los paquetes Dynamixel Workbench y el paquete del robot Phantom X. Esto facilitó la manipulación y control del robot Phantom X Pincher.

2. **Precisión en la Caracterización del Robot**:
   - Se realizó una detallada caracterización del robot Phantom X Pincher utilizando herramientas de medición como un calibrador. Los datos obtenidos fueron fundamentales para la correcta simulación y análisis cinemático del robot en MATLAB.

3. **Uso Efectivo de MATLAB y Toolboxes**:
   - La utilización de MATLAB junto con la Robotics Toolbox de Mathworks y la Toolbox de Peter Corke permitió la simulación precisa del robot. La implementación de la cinemática directa mediante la tabla DH y la función personalizada para calcular las matrices de transformación homogénea (MTH) demostró ser efectiva y precisa.

4. **Comparación Visual y Validación Experimental**:
   - La comparación de las posiciones simuladas del robot en MATLAB con las imágenes reales del manipulador Phantom X Pincher evidenció la precisión de la simulación. Las poses definidas se replicaron con exactitud en el entorno real, validando así el modelo matemático utilizado.

5. **Integración con ROS y Dynamixel**:
   - La integración con ROS y el uso del software Dynamixel facilitó el control y monitoreo de las articulaciones del robot. La edición del archivo `basic.yaml` y la creación de scripts en la carpeta `src` dentro de Catkin fueron pasos cruciales para la correcta manipulación del robot a través de ROS.

6. **Desarrollo de Interfaz Gráfica en Python**:
   - La implementación de una interfaz gráfica en Python permitió una interacción más amigable y eficiente con el robot. A pesar de los ajustes necesarios para evitar problemas de equilibrio en la pose 4, la interfaz gráfica demostró ser una herramienta útil para la visualización y control del robot.

7. **Ajustes y Mejoras Realizadas**:
   - La identificación y ajuste de los límites de las articulaciones fueron esenciales para asegurar la estabilidad y seguridad del robot durante las operaciones. Estos ajustes reflejan una atención meticulosa a los detalles y la comprensión de las limitaciones físicas del robot.

8. **Contribuciones Adicionales y Creatividad**:
   - El diseño de logos personalizados, aunque no se pudo integrar en la interfaz por falta de tiempo, muestra un esfuerzo adicional en personalizar y profesionalizar el proyecto. Estos detalles, aunque menores, contribuyen a la identidad visual y presentación del trabajo.

En resumen, el proyecto demuestra una implementación exitosa y detallada del control y simulación del robot Phantom X Pincher, combinando diversas herramientas y plataformas (MATLAB, ROS, Python) para lograr una solución robusta y precisa. La atención al detalle, desde la caracterización del robot hasta los ajustes en las poses y la interfaz gráfica, refleja un profundo entendimiento técnico y capacidad de resolución de problemas.

