[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-24ddc0f5d75046c5622901739e7c5dd533143b0c8e959d652212380cedb1ea36.svg)](https://classroom.github.com/a/6srJS4SU)
# P5 - La Estrella de la Muerte

Tenemos una Estrella de la Muerte que es capaz de realizar las siguientes acciones:

* Viajar de un punto a otro de la galaxia por el hiperespacio, siempre y cuando esos puntos estén conectados por una ruta hiperespacial.
* Destruir un planeta. Es posible que se necesiten varios disparos para destruir el planeta, pero se considera una acción única.

### Rutas del hiperespacio
Los planetas a los que podemos viajar son los siguientes, y las rutas hiperespaciales que los conectan quedan definidas por las flechas de la figura:

* Coruscant: Capital del imperio.
* Endor: Base de operaciones imperiales del borde exterior.
* Alderaan: Posible base rebelde.
* Dantooine: Posible base rebelde.
* Hoth: Posible base rebelde.
* Yavin4: Posible base rebelde.

![planets](https://github.com/Docencia-fmrico/2024-PSC-P5-DeathStar/assets/148446552/d819f783-48d5-49a7-9320-d3bdd9ebb4f6)


### Objetivo principal
El Comandante Supremo Lord Vader tiene la misión de destruir varios planetas en los que se cree que se esconden bases de la escoria rebelde.
Sin embargo, los deseos del comandante Vader son volátiles y cambian en cada momento, por lo que no es posible planificar una campaña estratégica de destrucción desde el momento inicial.
Por este motivo, no podemos generar un plan global para la estrella de la muerte en el que se secuencien todas las acciones de viajar y destruir planetas para destruir todos los planetas de la lista. Por lo tanto, se necesita diseñar un controlador que utilice plansys2 y que sea lo suficientemente flexible como para adaptarse a las órdenes de Vader. Este controlador deberá generar y ejecutar planes más sencillos que se encarguen de la destrucción de cada planeta de forma independiente.



## Antes de empezar
Este repositorio incluye un paquete de ROS 2 con la estructura necesaria para el sistema de planificación y ejecución de acciones de la Estrella de la Muerte. Debes clonarlo dentro de tu workspace y compilarlo. Analiza bien los componentes incluidos en el paquete y estudia cómo están implementados.

### Dominio PDDL
Se proporciona una implementación del dominio en PDDL para el problema propuesto en [pddl/deathstar.pddl](plansys2_deathstar_problem/pddl/deathstar.pddl). También se proporciona un problema de ejemplo y un archivo con comandos para la terminal de plansys2, pero éstos son únicamente para realizar pruebas sobre el PDDL.




## Ejercicio 1: Acciones
Implementa un nodo para ejecutar la acción `destroy_planet` en un fichero llamado [destroy_planet_action_node.cpp](plansys2_deathstar_problem/src/destroy_planet_action_node.cpp). Este nodo debe simular la acción de destruir un planeta, y tendrá las siguientes características:

* Frecuencia de actualización: 0.5Hz. Se realizarán llamadas al método `do_work` cada 2 segundos.
* Proceso de carga: En el primer "tick" de la acción, la estrella no podrá disparar, ya que primero tendrá que calentar el láser.
* Proceso de disparo: A partir del segundo tick, la estrella podrá realizar un disparo en cada tick. Cada disparo tendrá una probabilidad del 50% de destruir el planeta. La nave seguirá disparando hasta que el planeta haya sido destruido, y llevará una cuenta del número de disparos que ha realizado. El nodo mostrará por terminal un mensaje para cada disparo.
* La acción deberá enviar feedback en cada tick, y finalizará cuando el planeta haya sido destruido.

Indica cómo has implementado el nodo para esta acción y qué más archivos has tenido que modificar para instalarlo y ejecutarlo:

*[Respuesta]*  
Para la implementación de esta accion he distinguido entre dos fase de la accion con una varibale booleana que nos permite saber si es la primera vez que se hace tick a dicha accion y cuyo resultado sera cargar el laser y poner el progreso de la accion al 50%. Una vez pasada esa fase, diferenciamos entre otras dos subfases con un generador aleatorio de numero entre 0 y 1, si el numero obtenido es 0, se considera disparo fallido se mantiene el progreso a 50% y se manda el feedback correspondiente. Por el contrario se considera como disparo correcto y se escribe por pantalla el progressoo de 100% y utilizamos la funcion finish para dar por terminada dicha accion.  
En cada step de este nodo se imprime por pantalla dicho progreso.

Ademas de la creacion de este nodo hemos tenido que modificar el archivo CMakeList para añadir el ejectuble sus dependencias y añadirlo a la carpeta install.  
Por otra parte en el archivo launcher deberemos añadir dicho nodo para que este disponible en la ejecucion del programa.


**Nota**: Una vez implementada esta acción se puede probar el funcionamiento del sistema utilizando la terminal de plansys2 con los comandos disponibles en [pddl/commands.txt](plansys2_deathstar_problem/pddl/commands.txt).


## Ejercicio 3: Controlador
Implementa un nodo para controlar las acciones de la Estrella de la Muerte en un fichero llamado [deathstar_controller_node.cpp](plansys2_deathstar_problem/src/deathstar_controller_node.cpp). 

La lógica del controlador se guiará por los siguientes puntos:

1. Al inicio se preguntará a Lord Vader cuáles son sus indicaciones, y él dará la orden de destruir un planeta concreto, que será determinado de forma aleatoria entre los que siguan existiendo.
2. Una vez seleccionado el planeta, se actualizará el problema para definir ese nuevo objetivo (goal).
3. Después, se calculará el nuevo plan de acción de la Estrella de la Muerte y se ejecutarán las acciones del plan.
4. La ejecución del plan se detendrá cuando se cumpla el objetivo o si el plan falla.
5. Tras finalizar la ejecución del plan para destruir el planeta seleccionado, se volverá al punto 1 para destruir el siguiente planeta elegido aleatoriamente por Vader.
6. El programa terminará una vez destruidos todos los planetas rebeldes, comenzando por fin una nueva era de paz.


Recomendaciones:

* La lista de planetas rebeldes se puede almacenar dentro de la clase del controlador como un vector de strings, eliminando elementos del vector conforme van siendo destruidos los planetas.
* Una forma sencilla (aunque algo bruta) de seleccionar un elemento aleatorio de un vector y eliminarlo es utilizando la función [std::suffle](https://en.cppreference.com/w/cpp/algorithm/random_shuffle) para reordenar el vector aleatoriamente y extraer el último elemento con una llamada a `pop_back()`. Ejemplo:

    ```c++
    auto planets_ = std::vector<std::string> {"alderaan", "hoth", "yavin4"};
    std::string get_next_planet() {
        // Shuffle the whole list
        std::shuffle(planets_.begin(), planets_.end(), std::mt19937{std::random_device{}()});
        // Select the last one and remove it
        std::string planet_selected = planets_.back();
        planets_.pop_back();
        return planet_selected;
    }
    ```
* Se recomienda implementar una máquina de estados similar a la vista en clase del ejemplo [plansys2_patrol_navigation_example](https://github.com/PlanSys2/ros2_planning_system_examples/tree/rolling/plansys2_patrol_navigation_example). Recuerda que cuando no queden más planetas que destruir el controlador debe dejar de ejecutar acciones.

Una vez implementado, ejecúta el launch del sistema en una terminal y el controlador en otra y muestra los resultados:

*[Respuesta]*  


[Screencast from 03-18-2024 09:55:32 AM.webm](https://github.com/Docencia-fmrico/p5-deathstar-nicogmon/assets/102520722/f787757b-83f8-4a20-b6f6-fc2e60128ee6)



## Ejercicio Extra *[Opcional]*: Gestión de fallos
Ni siquiera el arma más avanzada de la galaxia es perfecta, y es posible que nuestros esfuerzos para destruir un planeta no sean suficientes.
Modifica el nodo de la acción `destroy_planet` para que tenga un número máximo de disparos. Si la estrella no ha conseguido destruir el planeta en menos de 5 disparos, la acción finalizará, pero con un resultado de fallo. Revisa la API de la clase [ActionExecutorClient](https://github.com/PlanSys2/ros2_planning_system/blob/rolling/plansys2_executor/src/plansys2_executor/ActionExecutorClient.cpp) para ver cómo finalizar la acción de esta forma. Puedes ajustar el número máximo de disparos y la probabilidad de destruir el planeta como consideres conveniente.

Indica qué cambios has tenido que realizar:

*[Respuesta]*

Después de modificar la acción para que pueda fallar, ajusta el controlador para que tenga en cuenta estos fallos. Si el plan de destruir un planeta falla, este planeta no podrá ser eliminado de la lista. Después de un fallo hay que suplicar a Lord Vader que nos perdone la vida y volver a preguntar por el siguiente objetivo.


Indica qué cambios has tenido que realizar y muestra el funcionamiento final:

*[Respuesta]*
