# Construcción de un Brazo Robótico mediante Código

Este tutorial describe los detalles de un objeto de modelo URDF y un ejemplo de la construcción de un brazo robótico de 3 articulaciones.

![](https://github.com/CarlosAlfredoMarin/Brazo_Robotico/blob/main/robot1_description/urdf/Brazo.png?raw=true)
>Brazo Robótico a construir

**Enlaces:** un enlace contiene las propiedades físicas de un cuerpo del modelo. Puede ser una rueda o un eslabón en una cadena conjunta. Cada enlace puede contener muchos elementos visuales y de colisión. Intente reducir la cantidad de enlaces en sus modelos para mejorar el rendimiento y la estabilidad. Por ejemplo, un modelo de mesa podría constar de 5 eslabones (4 para las patas y 1 para la parte superior) conectados mediante articulaciones. Sin embargo, esto es demasiado complejo, especialmente porque las articulaciones nunca se moverán. En su lugar, cree la tabla con 1 enlace y 5 elementos de colisión.

**Colisión:** un elemento de colisión encapsula una geometría que se utiliza para la comprobación de colisiones. Puede ser una forma simple (que se prefiere) o una malla triangular (que consume más recursos). Un enlace puede contener muchos elementos de colisión.

**Visual:** un elemento visual se utiliza para visualizar partes de un enlace. Un enlace puede contener 0 o más elementos visuales.

**Inercial:** el elemento inercial describe las propiedades dinámicas del enlace, como la masa y la matriz de inercia rotacional.

**Sensor:** un sensor recopila datos del mundo para usarlos en complementos. Un enlace puede contener 0 o más sensores.

**Luz:** un elemento de luz describe una fuente de luz adjunta a un enlace. Un enlace puede contener cero o más luces.

**Articulaciones:** una articulación conecta dos enlaces. Se establece una relación de padre e hijo junto con otros parámetros, como el eje de rotación y los límites de las articulaciones.

**Complementos:** un complemento es una biblioteca compartida creada por un tercero para controlar un modelo.




## Crear el Espacio de Trabajo

Crear una carpeta en el usuario raíz
<pre><code>mkdir -p ~/Brazo_Robot/src </code></pre>

Entramos a la carpeta
<pre><code>cd ~/Brazo_Robot/src </code></pre>

Inicializamos el espacio de trabajo
<pre><code>catkin_init_workspace </code></pre>

Salimos de la carpeta src
<pre><code>cd .. </code></pre>

Compilamos el espacio de trabajo
<pre><code>catkin_make </code></pre>

Observamos el listado de carpetas que se han creado
<pre><code>ls </code></pre>

Siempre que abramos un terminal nuevo debemos ejecutar un fichero de inicialización
<pre><code>echo "source ~/Brazo_Robot/devel/setup.bash" >> ~/.bashrc </code></pre>

Otra opción es crear un fichero de inicialización que ejecute ese comando automáticamente,

<pre><code>source ~/Brazo_Robot/devel/setup.bash </code></pre>

Ahora, empezamos a construir el robot, dentro de la carpeta src:
<pre><code>cd ~/Brazo_Robot/src </code></pre>  





## Construcción de las Partes de Robot

Creamos el paquete de desarrollo, el cual tiene un archivo con las librerías necesarias y la configuración. Las librerías que vamos a necesitar son *std\_msgs*, *rospy*, *roscpp* y*urdf*, luego iremos añadiendo las librerías necesarias para construir el robot. El paquete tendrá el nombre *robot1\_description*, este nombre debe estar todo en minúsculas para evitar futuros Warnings.


<pre><code>catkin_create_pkg robot1_description std_msgs rospy roscpp urdf
</code></pre>

Si se genera un error, para arreglarlo actualizamos las herramientas de catkin
<pre><code>sudo apt-get install python3-osrf-pycommon python3-catkin-tools
</code></pre>

Entramos a la carpeta *robot1\_description*
<pre><code>cd robot1_description
</code></pre>

Creamos una carpeta *urdf*
<pre><code>mkdir urdf</code></pre>

En la carpeta *urdf* es donde alojaremos los archivos de descripción del robot. El archivo <a href="https://github.com/CarlosAlfredoMarin/Brazo_Robotico/blob/main/robot1_description/urdf/robot1.urdf" target="_blank">robot1.urdf</a>  contiene la descripción de las siguientes partes del robot:

<ul>
    <li> Base (base_link)
    <li> Rueda 1 (wheel_1)
    <li> Rueda 2 (wheel_2)
    <li> Rueda 3 (wheel_3)
    <li> Rueda 4 (wheel_4)
    <li> Juntura Base-Rueda 1 (base_to_wheel1)
    <li> Juntura Base-Rueda 2 (base_to_wheel2)
    <li> Juntura Base-Rueda 3 (base_to_wheel3)
    <li> Juntura Base-Rueda 4 (base_to_wheel4)
    <li> Base de los eslabones (arm_base)
    <li> Juntura Base-Base de los eslabones (basse_to_arm_base)
    <li> Eslabón 1 (arm_1)
    <li> Eslabón 2 (arm_2)
    <li> Juntura Base-Eslabón 1 (arm_1_to_base)
    <li> Juntura Eslabón 1 - Eslabón 2 (arm_1_to_arm_2)
    <li> Pinza izquierda (left_gripper)
    <li> Pinza derecha (right_gripper)
    <li> Juntura Eslabón 2 - Pinza izquierda (left_gripper_joint)
    <li> Juntura Eslabón 2 - Pinza derecha (right_gripper_joint)
    <li> Punta izquierda (left_tip)
    <li> Punta derecha (right_tip)
    <li> Juntura Pinza izquierda - Punta izquierda (left_tip_joint)
    <li> Juntura Pinza derecha - Punta derecha (right_tip_joint)
</ul>  


Esta forma de describir el robot es ineficiente, ya que se repiten las estructuras de código varias veces, por ejemplo, para el código para crear la rueda 1 es casi el mismo que para las ruedas 2, 3 y 4. Por este motivo, resulta conveniente describir el robot de una forma más eficiente reutilizando bloques de código, esto se logra por medio de la etiqueta **xacro**, la cual funciona se manera semejante una función que recibe parámetros como entradas.

```xml
<xacro:macro name=" params="">
	...
</xacro:macro>
```

El archivo <a href="https://github.com/CarlosAlfredoMarin/Brazo_Robotico/blob/main/robot1_description/urdf/robot1_reduced.xacro" target="_blank">robot1_reduced.xacro</a> contiene el nuevo código reducido, el cual contiene:

<ul> 
    <li> Base (base_link)
    <li> Rueda (wheel)
    <li> Juntura Base-Rueda  (base_to_wheel)
    <li> Base de los eslabones (arm\_base)
    <li> Juntura Base-Base de los eslabones (basse_to_arm_base)
    <li> Eslabón (arm)
    <li> Juntura Base-Eslabón 1 (arm_1_to_arm_base)
    <li> Juntura Eslabón 1 - Eslabón 2 (arm_1_to_arm_2)
    <li> Pinza (lr_gripper)
    <li> Punta (lr_tip)
    <li> Juntura Eslabón - Pinza (gripper_joint)
    <li> Juntura Pinza - Punta (lr_tip_joint)
</ul>  



Existe una herramienta que permite ver el diagrama de conexiones del robot, usamos el siguiente comando para instalarla:

<pre><code>sudo apt install liburdfdom-tools
</code></pre>

ejecutamos la herramienta (debemos estar primero ubicados en la carpeta que contiene el archivo .urdf)
<pre><code>cd urdf
urdf_to_graphiz robot1.urdf
</code></pre>

Ahora, tenemos 2 archivos nuevos creados, en los cuales está la visualización gráfica de los link y joint, abrimos el archivo nuevo de extensión .pdf:


![](https://github.com/CarlosAlfredoMarin/Brazo_Robotico/blob/main/robot1_description/urdf/Diagrama_de_Conexiones.png?raw=true)
> Diagrama de conexiones


Para revisar si el archivo URDF tiene errores:
<pre><code>check_urdf robot1.urdf
</code></pre>

Es importante entrar a la carpeta donde se encuentra el archivos .urdf, de lo contrario, genera error.

Ahora, vamos a crear un archivo lanzador, con solo llamar este archivo por consola, se ejecutará automáticamente el *roscore* y se ejecutará Gazebo con nuestro robot en pantalla. Creamos una carpeta llamada *launch*:

<pre><code>/robot1_description/
mkdir launch
</code></pre>

Copiar el archivo <a href="https://github.com/CarlosAlfredoMarin/Brazo_Robotico/blob/main/robot1_description/launch/display.launch" target="_blank">display.launch</a>,
compilamos el espacio de trabajo, abrimos terminal en *Brazo_Robot*:

<pre><code>catkin_make
</code></pre>

Ejecutamos el archivo lanzador con el siguiente comando:

<pre><code>roslaunch robot1_description display.launch
</code></pre>

Para agregar un **mesh** existen varias formas de hacerlo:

Descargamos en la carpeta */Brazo_Robot/src* el mesh desde github o lo descargamos y lo movemos a la carpeta antes mencionada. </li>
        
<pre><code>git clone https://github.com/pr2/pr2_common.git
</code></pre>

Instalar los mesh directamente sobre ROS:
<pre><code>sudo apt-get install ros-noetic-pr2-common
</code></pre>

En nuestro caso se instaló la librería **pr2\_common** que contiene los meshes de un un efector final basado en 2 pinzas. Ahora que ya tenemos los meshes disponibles para su uso, solo queda agregar la línea de código para llamar al mesh, esto se hace dentro de la etiqueta ```<geometry>```

```xml
<geometry>
    <mesh filename="package://pr2_description/meshes/gripper_v0/l_finger.dae"/>
</geometry>
```

Esto se realiza para las 2 pinzas y para las 2 puntas de las pinzas.

Después compilamos el espacio de trabajo nuevamente
<pre><code>catkin_make
</code></pre>

Al ejecutar el lanzador, nuestro robot ya tendrá un efector final:

![](https://github.com/CarlosAlfredoMarin/Brazo_Robotico/blob/main/robot1_description/urdf/Efector_Final.png?raw=true)
>Efector final de brazo.



## Colores para el Brazo
Para pintar el robot con distintos colores es necesario agregar la siguiente línea de código al archivo <a href="https://github.com/CarlosAlfredoMarin/Brazo_Robotico/blob/main/robot1_description/urdf/robot1_reduced.xacro" target="_blank">robot1_reduced.xacro</a>:

```xml
<xacro:include filename="$(find robot1_description)/urdf/robot.gazebo"/>
```

Con esta nueva línea de código se está realizando una llamado al archivo <a href="https://github.com/CarlosAlfredoMarin/Brazo_Robotico/blob/main/robot1_description/urdf/robot.gazebo" target="_blank">robot.gazebo</a> el cual contiene los colores para el robot. Vamos a observar una parte del contenido de este archivo:

```xml
<gazebo reference="base_link">
    <material>Gazebo/Orange</material>
</gazebo>

<gazebo reference="wheel_1">
    <material>Gazebo/Black</material>
</gazebo>	
```

A través de la etiqueta ```<gazebo>``` se indica cuál ```Link``` del robot se pintará, y posteriormente con la etiqueta ```<material>``` se indica el color. En Gazebo ya existen colores predefinidos qeu pueden ser vistos en la página http://wiki.ros.org/simulator_gazebo/Tutorials/ListOfMaterials.

Esta página contiene una lista de los materiales disponibles para Gazebo. Esta lista puede variar de lo que tenga en su instalación. Puede consultar la lista de materiales disponibles actualmente haciendo

<pre><code>gedit 'rospack find gazebo'/gazebo/share/gazebo/Media/materials/scripts/Gazebo.material
</code></pre>






## Controlador Básico para el Brazo

Para que una articulación pueda ejercer algún movimiento es necesario asignar un controlador a cada articulación, posteriormente, se publica en el tópico correspondiente a la articulación que deseamos mover. Para lograr ello, se requieren 2 archivos nuevos:

<a href="https://github.com/CarlosAlfredoMarin/Brazo_Robotico/blob/main/robot1_description/config/robot1_gazebo_control_Position.yaml" target="_blank">robot1_gazebo_control_Position.yaml</a>

<a href="https://github.com/CarlosAlfredoMarin/Brazo_Robotico/blob/main/robot1_description/src/Mover_Articulacion.py" target="_blank">Mover_Articulacion.py</a>

Revisaremos el archivo <a href="https://github.com/CarlosAlfredoMarin/Brazo_Robotico/blob/main/robot1_description/config/robot1_gazebo_control_Position.yaml" target="_blank">robot1_gazebo_control_Position.yaml</a>:

```yml
joint1_position_controller:
    type: effort_controllers/JointPositionController
    joint: base_to_arm_base
    pid: {p: 100.0, i: 0.01, d: 10.0}
```

En esta sección de código, se tiene 1 controlador para 1 articulación. El nombre del controlador es ```joint1_position_controller```; el parámetro ```type``` indica el tipo de controlador, este controlador es de tipo ```effort``` o ```esfuerzo```; el parámetro ```joint``` indica a qué juntura o articulación estará ligado el controlador, la articulación a la que está asignado este controlador es ```base_to_arm_base```. El parámetro ```pid``` permite asignar ganancias al controlador.

De la misma forma se realiza para todas las ```junturas``` o ```joints``` del brazo robótico.

Ahora, revisaremos el archivo <a href="https://github.com/CarlosAlfredoMarin/Brazo_Robotico/blob/main/robot1_description/src/Mover_Articulacion.py" target="_blank">Mover_Articulacion.py</a>:

```cpp
pub = rospy.Publisher('/robot1/joint3_position_controller/
command', Float64, queue_size=10)
rospy.init_node('talker', anonymous=True)
rate = rospy.Rate(10)
position = 3.141592
pub.publish(position)
```

En este archivo se crea un publicador llamado ```pub```, el cual publica en el tópico ```/robot1/joint3_position_controller/command``` un mensaje de tipo ```Float64```. La variable ```position``` almacena la posición deseada, la cual debe estar en ```radianes```, finalmente, con la línea de código ```pub.publish(position)``` se publica la posición deseada y la articulación correspondiente se mueve hasta el ángulo deseado.

La Figura muestra la posición alcanzada por el segundo eslabón del robot cuando la entrada de posición es $$\pi/2$$, donde el origen angular es sobre el eje vertical, es decir en $$+90^\circ$$.

![](https://github.com/CarlosAlfredoMarin/Brazo_Robotico/blob/main/robot1_description/urdf/Brazo_Posicion.png?raw=true)
>Posición alcanzada por el brazo.

Dado que las demás articulaciones ya tienen configurado un ```controlador de esfuerzo``` el robot mantiene su postura firme, antes de la implementación de los controladores, el robot se caía al suelo, porque no tenía firmeza, la cual se la otorgan los ```controladores de esfuerzo```.
