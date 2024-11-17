[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-24ddc0f5d75046c5622901739e7c5dd533143b0c8e959d652212380cedb1ea36.svg)](https://classroom.github.com/a/mA4RGlJr)
# Trabajo Final Planificación

El trabajo final consiste en implementar en equipo un sistema de planificación para un robot que realice tareas del hogar.
El robot será capaz de realizar una serie de tareas (misiones) que pueden estar compuestas por una o más acciones. Como mínimo el robot debe poder ejecutar 3 tareas distintas.
A continuación se muestra un ejempo de posibles tareas a ejecutar:

* Hacer la cama
* Lavar los platos
* Poner la mesa
* Atender visitas
* Recoger la basura

Se dispone de libertad para elegir el entorno en el que trabaje el robot, con las siguientes condciones:

* La casa debe disponer de varias habitaciones y de al menos una puerta, que podrá estar abierta o cerrada. El robot será capaz de abrir la puerta o de solicitar a un amigo humano que la abra.
* Se deben definir varios waypoints en el mapa de la casa para determinar a qué puntos puede navegar el robot. Además, es recomendable definir waypoints en las entradas y salidas de las habitaciones para interactuar con las puertas.

## Antes de empezar
La primera tarea es decidir en qué entorno va a operar el robot y qué tareas será capaz de realizar.

Después, se deberá crear un paquete de ROS 2 donde se incluirá todo el código generado para esta práctica, incluyendo la implementación de las acciones, las definiciones PDDL y los archivos de configuración necesarios para ejecutarlo todo.

## Ejercicio 1 - Dominio PDDL

Implementar un dominio en PDDL en el que se modele el mundo en el que va a operar el robot y las distintas acciones que este puede ejecutar. Hay que tener en cuenta que una tarea puede estar compuesta por una o varias acciones. Por ejemplo, para lavar los platos puede que sean necesarias las acciones de moverse, cruzar una puerta, y lavar los platos.

Indicad la lista de acciones implementadas y la lista de tipos, predicados y fluents necesarios para vuestro modelo:

*[Respuesta]*

#### *Acciones:*
    move  
    ask_open_door  
    pass_door  
    tie_bed  
    dish_wash  
    attend_person  
    pick_up_trash  
    setup_table  

#### *Types*  
    robot  
    door -object  
    door_wp - waypoint  
    bed - object  
    trash - object  
    table - object  
    dishes - object  
    person  
#### *Predicates*
    (robot_at ?r - robot ?w - waypoint)  
    (connected ?w1 - waypoint ?w2 - waypoint)  
    (door_at ?d - door ?w - waypoint)  
    (door_closed ?d - door)  
    (door_opened ?d - door)  
    (object_at ?o - object ?w - waypoint)  
    (person_at_door ?p - person ?w - waypoint ?d - door)  
    (bed_tied ?b - bed)  
    (table_setup ?t - table)  
    (trash_pickup ?t - trash)  
    (person_attended ?p - person ?dest - waypoint)  
    (cleaned_dishes ?d - dishes)  

## Ejercicio 2 - Acciones

Implementar las distintas acciones en PlanSys2 como nodos de BehaviorTree. Se pueden utilizar los nodos implementados en [plansys2_bt_example](https://github.com/PlanSys2/ros2_planning_system_examples/tree/rolling/plansys2_bt_example) como ejemplo.

**Importante:** La acción de navegar para que el robot se mueva de un punto a otro debe utilizar el sistema de navegación de nav2. Es decir, esta acción deberá realizar llamadas a las acciones de nav2 para que el robot se mueva de un punto a otro, de una forma similar a la implementada en los ejemplos [plansys2_bt_example](https://github.com/PlanSys2/ros2_planning_system_examples/tree/rolling/plansys2_bt_example) y [plansys2_patrol_navigation_example](https://github.com/PlanSys2/ros2_planning_system_examples/tree/rolling/plansys2_patrol_navigation_example). Si no se utiliza un simulador real (Ejercicio 4) se deberá utilizar el nodo `nav2_sim_node` utilizado en los [ejemplos](https://github.com/PlanSys2/ros2_planning_system_examples/blob/4c4fe955f7d8cd36b6f1ca92aead728a595cfc76/plansys2_patrol_navigation_example/launch/patrol_example_fakesim_launch.py#L37-L42) de plansys2 para simular la navegación del robot.

El resto de acciones a realizar por el robot pueden ser sintéticas, donde la acción habrá terminado después de que haya pasado un tiempo determinado.

### Composición de acciones con nodos de BT
Las acciones definidas en el dominio PDDL son equivalentes a un árbol de ejecución de BT. Ese árbol puede utilizar varias sub-acciones más pequeñas, que se implementarán como nodos del árbol. Por ejemplo, la acción de lavar los platos puede consistir en la secuencia de sub-acciones [abrir el grifo --> coger la esponja --> fregar --> enjuagar --> soltar esponja --> cerrar grifo]. Cada una de estas sub-acciones serán implementadas como un nodo de BehvaiorTree.

Indicar cómo han sido definidas las acciones en el paquete y qué sub-acciones (nodos BT) han sido implementadas:

*[Respuesta]*

Las acciones definidas que estan compuestas de subacciones son  attend_person, tie_bed, dish_wash, pick_up_trash y setup_table.

Estos nodos tendrán la siguiente estructuras BT:

**Sub-accines attend_person:**

* Open_door
* Ask_destination
* present_person

![attendPerson](https://github.com/Docencia-fmrico/trabajo-plansys2-cero_dumped/blob/main/images/attendPerson.jpeg)

**Sub-acciones tie_bed:**

* stretch_sheet
* place_blanket
* pillow-placement

![tie_bed](https://github.com/Docencia-fmrico/trabajo-plansys2-cero_dumped/blob/main/images/tieBed.jpeg)

**Sub-acciones dish_wash:**

* open_tap
* grab_songe
* scrub
* rinse
* drop_sponge
* close_tap

![dishWash](https://github.com/Docencia-fmrico/trabajo-plansys2-cero_dumped/blob/main/images/dishWash.jpeg)

**Sub-acciones pick_up_trash:**

* pick-trash
* drop_trash

![pickUpTrash](https://github.com/Docencia-fmrico/trabajo-plansys2-cero_dumped/blob/main/images/pickUpTrash.jpeg)

**Sub-acciones setup_table:**

* place_cutlery
* place_dishes
* place_glasses

![setupTable](https://github.com/Docencia-fmrico/trabajo-plansys2-cero_dumped/blob/main/images/setupTable.jpeg)

Aquellas acciones que solo cuentan con un nodo principal son ask_open_door, move y pass_door. Estos *behavior trees* tienen una estructura simple:

**ask_open_door:**
![askOpenDoor](https://github.com/Docencia-fmrico/trabajo-plansys2-cero_dumped/blob/main/images/askOpenDoor.jpeg)

**move:**
![move](https://github.com/Docencia-fmrico/trabajo-plansys2-cero_dumped/blob/main/images/Move.jpeg)

**pass_door:**
![passDoor](https://github.com/Docencia-fmrico/trabajo-plansys2-cero_dumped/blob/main/images/passDoor.jpeg)

## Ejercicio 3 - Planner
Utilizar un planificador distinto a POPF o TFD para generar los planes. Para conseguir esto es necesario implementar un plugin para poder llamar al planificador elegido desde plansys2. Este plugin se debe crear en un **paquete de ROS 2 aparte**, y consistirá en una clase que herede de [PlanSolverBase](https://github.com/PlanSys2/ros2_planning_system/blob/rolling/plansys2_core/include/plansys2_core/PlanSolverBase.hpp). Se puede utilizar la implementación del [plugin de POPF](https://github.com/PlanSys2/ros2_planning_system/tree/rolling/plansys2_popf_plan_solver) como referencia.

**Nota:** Si es posible, se deberá implementar un plugin para el planificador escogido por el grupo en el trabajo de PDDL. El planificador deberá soportar al menos `durative-actions`, por lo que si se escogió un planificador no compatible tendréis que elegir otro para implementar el plugin de plansys2.

Plugin escogido y detalles de cómo poder usarlo en plansys:

*[Respuesta]*


## Ejercicio 4 - Simulador real
Utilizar un simulador basado en gazebo de un robot real para realizar las pruebas. Se puede utilizar cualquier simulador, aunque se recomiendan los simuladores del [TIAGO](https://github.com/jmguerreroh/tiago_simulator) o del [Kobuki](https://github.com/IntelligentRoboticsLabs/kobuki), que ya se han utilizado en otras asignaturas.

Al utilizar un robot de verdad, las tareas de navegación deberán utilizar nav2 para que el robot se mueva de verdad. El resto de las acciones pueden seguir siendo sintéticas. Es decir, no es necesario que el robot interactúe con los objetos de la casa.

Se dispone de libertad para elegir el mundo de gazebo, siempre y cuando se cumplan las condiciones mencionadas anteriormente. No es necesario que las puertas se abran y cierren físicamente, pero sí que en la base de conocimiento del robot cambie el estado de la puerta después de abrirla o cerrarla.

Indicar qué simulador se ha utilizado, cómo ejecutarlo y cómo lanzar el sistema de planificación para que actúe sobre el robot:

*[Respuesta]*

El simulador utilizado para la realización de la práctica ha sido el del [Kobuki](https://github.com/IntelligentRoboticsLabs/kobuki). Los comandos necesarios para iniciar la ejecución del sistema son los siguientes:

* **Simulador**
```bash
ros2 launch kobuki simulation.launch.py x:=8.0 y:=-4.0 Y:=-1.57
```
* **Navegación**
```bash
ros2 launch kobuki navigation_sim.launch.py
```
* **Navegación Sinteica**
(Utilizar este comando en caso de no querer lanzar la simulación y navegación real. No se deben lanzar los dos anteriores en este caso.)
```bash
ros2 run plansys2_house_problem nav2_sim_node
```
* **Paquete de Plansys2**
```bash
ros2 launch plansys2_house_problem house_problem_launch.py 
```

* **Controlador**
```bash
ros2 run plansys2_house_problem house_controller_node
```

Estos comandos deben ejecutarse en terminales independientes. Es recomendable esperar a que finalice la ejecución de cada uno antes del siguiente.

## Vídeo final
Para finalizar, se debe incluir un vídeo del sistema funcionando, en el que se pueda apreciar el desarrollo realizado.

*[Vídeo]*
[Simulación_CeroDumped](https://urjc-my.sharepoint.com/:v:/g/personal/a_martinezal_2021_alumnos_urjc_es/ETMgmyYQSgxOhyJPZAWb4WcBoOZI4pc-p7ZhZp5dpaE6YA?nav=eyJyZWZlcnJhbEluZm8iOnsicmVmZXJyYWxBcHAiOiJPbmVEcml2ZUZvckJ1c2luZXNzIiwicmVmZXJyYWxBcHBQbGF0Zm9ybSI6IldlYiIsInJlZmVycmFsTW9kZSI6InZpZXciLCJyZWZlcnJhbFZpZXciOiJNeUZpbGVzTGlua0NvcHkifX0&e=dMZ8cq) 



