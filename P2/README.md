[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-24ddc0f5d75046c5622901739e7c5dd533143b0c8e959d652212380cedb1ea36.svg)](https://classroom.github.com/a/x2hg7_ib)
# P2 - Depósito

El propósito de esta práctica es mejorar el modelo creado en la práctica anterior, introduciendo nuevas features de PDDL como `:fluents` y `:durative-actions`.

Cansados de que los robots se estén moviendo constantemente de un lado a otro recogiendo basura, hemos creado un nuevo robot recolector de basura que tiene una pinza y un pequeño contenedor donde puede ir depositando la basura que recoge. Como este contenedor tiene una capacidad limitada, el robot viene con un depósito más grande que se ha instalado en un punto fijo de la habitación. Si el contenedor del robot está lleno, o ya no puede cargar más objetos, el robot debe dirigirse al depósito principal para descargar la basura que tenga en su pequeño contenedor.

## Ejercicio 1
Implementa desde cero el problema propuesto usando `:fluents`y aciones normales. Crea un fichero llamado [deposit_fluent_domain.pddl](deposit_fluent_domain.pddl) para la definición del dominio y otro llamado [deposit_problem.pddl](deposit_problem.pddl) para la definición del problema.

### Dominio
El dominio debe incluir un set de `functions` ([numeric fluents](https://planning.wiki/ref/pddl21/domain#numeric-fluents)) para definir las siguientes variables.

* Capacidad máxima del contenedor del robot.
* Carga actual del contenedor del robot.
* El peso de un objeto.

Además, el robot es capaz de realizar las siguientes acciones:

* **move**: El robot se mueve de un punto a otro.
* **pick**: Acción de recoger. El robot recoge un objeto con la pinza. El objeto pasa a estar siendo agarrado por la pinza, pero no cargado en el contenedor.
* **drop**: Acción de soltar. El robot suelta el objeto de la pinza en una ubicación.
* **load**: Acción de cargar. El robot carga un objeto en su contenedor. El objeto debe estar siendo agarrado por la pinza.
* **unload**: Acción de descargar. El robot descarga un objeto en el depósito.

**Nota**: Un objeto debe ser agarrado por la pinza antes de poder cargarlo en el contenedor del robot.

Describe aquí la lista de predicados y funciones necesarios para este modelo, explicando brevemente el propósito de cada uno.


### functions:
**(container_max_capacity ?r - robot ?c - container)**    
Fluent que nos permite saber la carga maxima del contenedor asi pudiendo saber si podemos cargar mas objetos o no.  

**(container_current_load ?r - robot ?c - container)**   
Fluent que nos permite saber la carga actual del contenedor igualmente util para controlar la carga del contorolador y saber si podemos cargarlo mas.  

**(weight ?it - item)**    
Fluent qu enos permite saber el peso de un objeto concreto y utilizado para calcular el peso del contendor actual y el posible peso en caso de cargar algo al contenedor

### predicates:
  **(robot_at ?r - robot ?l - location)**  
  Localización del robot.  
  
  **(item_at ?it - item ?l - location)**  
  Localización del objeto.  
  
  **(gripper_free ?r - robot ?g - gripper)**  
  Pinza g libre perteneciente a robot r.    
  
  **(gripper_carry ?r - robot ?g - gripper ?it - item)**  
  Pinza g de robot r cogiuend esta coguiendo el objeto it.  
  
  **(gripper_from  ?r - robot ?g - gripper)**  
  Pinza g pertenece al robot r.  
  
  **(object_in_container ?it - item ?r - robot ?c - container)**  
  El objeto it esta en el contenedor c del robot r.  
  
  **(container_from ?r - robot ?c - container)**  
  El contenedor c es del robot r.  
  
  


### Problema
El archivo de definición del problema debe incluir un único robot, y una lista de objetos (sólo basura) y ubicaciones (suelo, mesa, cama, etc.).
También debes inicializar el peso de cada objeto y la capacidad del contenedor del robot con los valores que consideres apropiados.

Finalmente, el objetivo final se debe fijar a que todos los objetos acaben descargados en el depósito principal.

Ejecuta el planificador (POPF / OPTIC) con los archivos de dominio y problema implementados y analiza la salida.
¿Está cargando el robot varios objetos en su contenedor antes de ir al depósito a descargar? ¿Está cargando y descargando los objetos de forma individual? ¿Por qué crees que se comporta de esta manera?

En este primer apartado el robot coge objetos de uno en uno y los lleva al contenedor y de hecho inicialmente el robot ignoraba su contendor daod que siendo el coste de las tareas el mismo era un coste añadido guardar las cosas en el contenedor pudiendo viajar con ellas en la pinza y por tanto tuve que añadir una precondicion a la accion de movimiento que especificara que la pinza debia esatr vacia para poder usar la accion move, aun asi al ser igual de costosos el robot coge un objeto lo guarda se va al contenedor y lo descarga y suelta.


Ahora, modifica la capacidad máxima de carga del contenedor del robot y/o modifica el pero de los objetos, y vuelve a ejecutar el planificador. ¿Cambia el plan según lo esperado?

Independientemente de que el peso de los objetos o la capcidad maxima cambien el planificador acaba alcanzando el mismo plan de coger los objetos cargarlos y llevarlos al deposito de uno en uno.


## Ejercicio 2
Para tener un modelo más realista, en este segundo ejercicio se deben cambiar las acciones normales por [durative actions](https://planning.wiki/ref/pddl21/domain#durative-actions).
Para esto, crea un nuevo archivo de dominio llamado [deposit_durative_domain.pddl](deposit_durative_domain.pddl). Puedes reutilizar el anterior archivo de problema.

Dispones de libertad para elegir la duración de cada una de las acciones, pero la acción `move` debe ser durar significantemente más que el resto.

Ejecuta el planificador con las nuevas acciones durativas y analiza la salida.
¿Está cargando el robot varios objetos en su contenedor antes de ir al depósito a descargar?

Con el planificador optic planer se obtiene varios planes en algunos de ellos es capz de coger mas de un objeto y guardarlo en su contenedor  en el caso de que hay mas de un objeto en la misma localizacion. 

¿Cuáles son las diferencias en la salida del planificador con y sin `durative actions`?

La diferencia principal es precisamente el hecho de guardar mas de un objeto a la vez ya que ahora el moverse requiere de un mayor coste temporal que coger y cargar objetos en el contenedor pequeño. Aun asi en la mayoria de planes solo recoge mas de una cosa si estas estan ubicados en el mismo lugar.

¿Qué diferencia hay entre la salida del planificador POPF y OPTIC? ¿Cuál parece funcionar mejor?

Observamos que popf detecta el coste temporal de las acciones pero no es capaz de planificar teniendo en cuenta este coste y tratando de minimizarlo, podriamos decir que planififca tomando como objetivo el acercarse al objetivo final lo maximo posible con cada movimineto en vez de tener en cuenta que podría conseguirlo en menor tiempo combinando pasos.
En este caso OPTIC parece encontrar un mejor plan aunque con algunas acciones un poco extrañas en algunos casos.


## Ejercicio extra [*Opcional*]
Incluye una función para expresar la distancia entre dos ubicaciones, y modifica la duración de la acción `move` para que dependa de esta distancia.
Como es necesario modificar tanto el dominio como el problema, puedes crear dos nuevos ficheros [deposit_domain_extra.pddl](deposit_domain_extra.pddl) y [deposit_problem_extra.pddl](deposit_problem_extra.pddl) para este ejercicio extra.
