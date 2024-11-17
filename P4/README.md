[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-24ddc0f5d75046c5622901739e7c5dd533143b0c8e959d652212380cedb1ea36.svg)](https://classroom.github.com/a/oNmmglua)
# P4 - Introducción a PlanSys2

El propósito de esta práctica es familiarizarse con el uso de PlanSys2, usando la terminal para definir el problema e implementando acciones sencillas en ROS 2.

Para ello, en este repositorio deberás crear un paquete de ROS 2 para implementar las acciones necesarias y para poder lanzar el sistema mediante `ros2 launch`. Es recomendable basarse en el ejemplo sencillo que se encuentra en [plansys2_simple_example](https://github.com/PlanSys2/ros2_planning_system_examples/tree/rolling/plansys2_simple_example).


## Ejercicio 1
Crea un paquete de ROS 2 basado en CMake con nombre `plansys2_p5` que incluya al menos las siguientes dependencias:

* rclcpp
* plansys2_msgs
* plansys2_executor
* plansys2_bringup (sólo ejecución)
* plansys2_terminal (sólo ejecución)

Indica qué comandos has utilizado para crear el paquete, y si has copiado el paquete de ejemplo de [plansys2_simple_example](https://github.com/PlanSys2/ros2_planning_system_examples/tree/rolling/plansys2_simple_example), indica qué cosas has cambiado en el paquete para adaptarlo.

· Nombre del paquete en package.xml  "<name>plansys2_p5</name>"  
· Nombre del proyecto en cmakelist project(plansys2_p5)   
. Instrucciones add_executable para los nodos de las acciones del dominio  
. Añadido de nodso a la instruccion install del cmakelist  
. Creacion de los nodos en el launcher y cambio de nombre de paquete en las descripciones de los nodos  




## Ejercicio 2
Añade el código PDDL del dominio del problema de la P2 (Depósito) a la carpeta `pddl/`. Recuerda que este dominio debe definir al menos las siguientes acciones:

* **move**: El robot se mueve de un punto a otro.
* **pick**: Acción de recoger. El robot recoge un objeto con la pinza. El objeto pasa a estar siendo agarrado por la pinza, pero no cargado en el contenedor.
* **drop**: (Opcional) Acción de soltar. El robot suelta el objeto de la pinza en una ubicación.
* **load**: Acción de cargar. El robot carga un objeto en su contenedor. El objeto debe estar siendo agarrado por la pinza.
* **unload**: Acción de descargar. El robot descarga un objeto en el depósito.

Después, añade un fichero [deposit_example_launch.py](launch/deposit_example.launch.py) a la carpeta `launch`, o modifica el fichero del ejemplo, para que utilice el archivo PDDL con el dominio que acabas de incluir en el paquete.

**Nota:** La carpeta `pddl/` debe ser marcada en el `CMakeLists.txt` para ser instalada.

## Ejercicio 3
Para cada una de las acciones definidas en el dominio, es necesario implementar un nodo de ROS 2 en un archivo `.cpp` dentro de la carpeta `src/`. De momento, la implementación de las acciones no debe hacer nada complejo. Es suficiente con hacer un contador que haga terminar la acción en x segundos, siguiendo la implementación que puedes encontrar en las acciones del ejemplo propuesto.

Para cada acción es necesario realizar lo siguiente:

1. Implementar el nodo de ROS 2 en una clase que herede de `plansys2::ActionExecutorClient`.
2. Añadir las instrucciones necesarias al `CMakeLists.txt` para que se genere el ejecutable de la acción y se instale.
3. Añadir la acción al fichero [deposit_example.launch.py](launch/deposit_example.launch.py) para que se ejecute al lanzarlo todo junto.


## Ejercicio 4
Una vez esté todo implementado, configurado y compilado/instalado, es posible lanzar el sistema con la siguiente instrucción:

```bash
ros2 launch plansys2_p5 plansys2_p5_launch.py
```

En otra terminal, abre la terminal de plansys con el siguiente comando:

```bash
ros2 run plansys2_terminal plansys2_terminal
```

En la terminal de plansys debes ejecutar los comandos necesarios para definir el problema. Puedes basarte en el problema de la Práctica 2.
Indica qué comandos has introducido en la terminal de plansys para definir el problema (objetos, predicados, etc.) y para definir el objetivo (goal) del problema:
```bash
# defincion de objetos, robots y localizaciones
set instance walle robot
set instance table location
set instance floor location
set instance fridge location
set instance shelving location
set instance big_container deposit
set instance bottle item 
set instance newspaper item
set instance rotten_apple item
set instance book item
set instance banana item
set instance wg1 gripper
set instance wc1 container

set predicate (robot_at walle big_container)
set predicate (gripper_from walle wg1)
set predicate (gripper_free walle wg1)
set predicate (container_from walle wc1)
set predicate (item_at bottle floor)
set predicate (item_at newspaper table)
set predicate (item_at rotten_apple fridge)
set predicate (item_at book shelving)
set predicate (item_at banana fridge)



set function (= (container_max_capacity walle wc1) 8)
set function (= (container_current_load walle wc1) 0)
set function (= (weight bottle) 1)
set function (= (weight newspaper) 1)
set function (= (weight rotten_apple) 1)
set function (= (weight book) 1)
set function (= (weight banana) 1)

set goal (and (item_at bottle big_container)(item_at rotten_apple big_container)(item_at newspaper big_container )(item_at book big_container)(item_at banana big_container))

```
