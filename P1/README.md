[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-24ddc0f5d75046c5622901739e7c5dd533143b0c8e959d652212380cedb1ea36.svg)](https://classroom.github.com/a/GCh42miP)
# P1 - Reciclaje

El propósito de esta práctica es modelar un entorno dentro de un edificio, donde un robot se dedica a coger objetos del suelo y organizarlos.
Los objetos (items) pueden ser basura, en cuyo caso el robot los debe soltar en el cubo de basura que le corresponda.
Los objetos que no sean basura deben ser colocados en una mesa.

## Ejemplo sencillo
Los archivos [recycling_domain_example](recycling_domain_example.pddl) y [recycling_problem_example](recycling_problem_example.pddl) contienen un dominio y un problema de ejemplo con la funcionalidad más básica.

En este ejemplo, se asume que el robot dispone de una base móvil con una pinza (gripper) con la capacidad de coger y soltar objetos.

Además, el dominio propuesto se limita a una única habitación, donde asumimos que el robot se puede mover libremente de una ubicación a otra.


**Ejercicio de calentamiento 1:** Analiza el ejemplo propuesto y ejecuta el planificador (POPF) para ver el plan que genera. Puedes cambiar la definición del problema para ver cómo cambian los planes.

*Nota: Puedes encontrar instrucciones para instalar y ejecutar POPF en el archivo [POPF_INSTRUCTIONS](POPF_INSTRUCTIONS.md).*

**Ejercicio de calentamiento 2:** Elimina o comenta la línea 14 del archivo [recycling_problem_example](https://github.com/Docencia-fmrico/2024-PSG-P1-Recycling/blob/afa78808a01408df736fdde3f0e965725673bf67/recycling_problem_example.pddl#L14), de tal forma que el robot ya no se encuentre inicialmente junto a la mesa:

```pddl
; (robot_at walle table)
```

Ahora vuelve a ejecutar el planificador y comprueba qué ocurre. ¿Por qué hay un error en la planificación? ¿Crees que sería posible que se mueva el robot sin saber su ubicación inicial?

Ocurre un error en la planificacion dado que el planificador intenta ejecutar acciones de movimiento del robot que tienen como precondicion que el robot este en algun sitio y para ello tiene que haber una sentencia que asi lo indique al eliminar dicha sentencia del conocimiento inicial el robot esta completamente desubicado y no podra ejecutar la acccion move en ninguna posicion.  
En ejemplos como el descrito la unica opción seria no considerar si el robot esta en una posicion o no y simplemente hacerlo avanzar a la posicion deseada lo cual complicaria la propia accion de movimiento del robot teniendo que usar una navegacion mas compleja ya que en estos casos se trata como una navegacion preprogramada entre dos ubicaciones y no entre cualquier punto del mapa y por lo que ya deberiamos dotar al robot de sensores y un mapa que le ayuden en la localizacion y navegacion al objetivo, o en caso de un mundo no realista que la accion de movimiento se realizara de forma instantanea simplemente asumiendo que la posicion del robot ha cambiado a la deseada 



**Ejercicio de calentamiento 3:** Modifica el problema para añadir más objetos (basura o no), e incluye un segundo robot que ayude a Wall-e con la limpieza.

Responde a la siguiente pregunta: ¿Es necesario que un objeto se encuentre inicialmente en el suelo? Tal y como está definido el dominio del ejemplo, ¿podríamos inicializar el problema con basura encima de la mesa? Pruébalo modificando el archivo del problema.

Dadas las precondiciones de los diferentes metodos especificadas en el dominio no es necesario que los objetos se encuentren inicialmente en el suelo, de hecho si los objetos se encuentran inicialmente en la ubicacion del robot este podra cogerlos directamente y despues desplazarse al sitio correspondiente.

## Ejercicio 1
Hemos actualizado nuestros robots de limpieza para que tengan la capacidad de llevar hasta 2 pinzas a la vez. Con estos nuevos robots, el dominio que tenemos de ejemplo ya no nos sirve. Indica el porqué.
Después de haber analizado las limitaciones que tiene el dominio de ejemplo con nuestros nuevos robots, crea un nuevo dominio en un archivo llamado [recycling_domain_gripper.pddl](recycling_domain_gripper.pddl) en el que se modele este cambio. Recuerda que ahora un robot puede tener más de una pinza, y que los objetos ya no serán manipulados por el robot directamente, sino por la pinza (que está conectada a la base del robot).

El dominio usado para el ejemplo  de una pinza ya no nos sirve dado que ahora tendremos un nuevo tipo de objeto que seran las pinzas ya que necesitaremos especeficar en la accion de coger o soltar un objeto que pinza realiza dicho movimiento a parte de a que robot pertenecen estas lo cual ya se especificaba anteriormente. Ademas deberemos llevar cuenta de si dichas pinzas estan ocupadas o libres  y en este caso tambien debermeos poder especificar de que pinza exactamente se trata.   

Por tanto añadimos las siguientes lineas o modificaciones:
Nuevo tipo gripper
```pddl
(:types
  ...
  gripper
)
```
Modificamos predicados para añadir la especificacion de pinza y especificar que el objeto lo lleva la pinza y no el robot
```pddl
(:predicates
  ...
  (gripper_free ?r - robot ?g - gripper)
  (gripper_carry ?r - robot ?g - gripper ?it - item)
)
```
Por último en acciones pick y drop-off modificamos los paramtros añadiendo la pinza que ejecuta:
```pddl
:parameters (?it - item ?l - location ?r - robot ?g - gripper)
```



## Ejercicio 2
Ahora, crea un nuevo problema (puede estar basado en el problema de ejemplo) en un archivo llamado [recycling_problem_gripper.pddl](recycling_problem_gripper.pddl). El problema debe contener al menos dos robots y varios objetos de distintos tipos (basura y no basura). Al menos uno de los robots debe tener 2 pinzas
El problema se basa en el problema de ejmeplo pero añadiendo otras tarea aparte del reciclaje que seria el ordenado de otros articulos o la limpieza de la nevera de comida en mal estado como la banana.
Ejecuta el planificador con los nuevos archivos de dominio y problema y copia aquí la salida. ¿Es el resultado como esperabas?  
  
El resultado es el esperado el robot actua de manera bastante eficiente recogiendo mas de un objeto en caso de estar en el mismo sitio o incluso cogiendo de diferentes sitios, el  funcionamiento es el que realizaria seguramente una persona o muy parecido.
```pddl
(:goal (and
    (item_at bottle table)
    (item_at rotten_apple organic_trashbin)
    (item_at newspaper table)
    (item_at book shelving)
    (item_at banana organic_trashbin)
  )
)
```


```pddl
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
b (30.000 | 0.000)b (28.000 | 0.000)b (26.000 | 0.001)b (24.000 | 0.002)b (22.000 | 0.003)b (20.000 | 0.004)b (18.000 | 0.004)b (16.000 | 0.004)b (14.000 | 0.004)b (12.000 | 0.004)b (10.000 | 0.004)b (8.000 | 0.005)b (6.000 | 0.006)b (4.000 | 0.007)b (2.000 | 0.008);;;;
Solution Found
; Time 0.00
0.000: (move walle table floor)  [0.001]
0.000: (pick banana fridge eva eg1)  [0.001]
0.001: (move eva fridge paper_trashbin)  [0.001]
0.001: (pick bottle floor walle wg1)  [0.001]
0.002: (pick rotten_apple paper_trashbin eva eg2)  [0.001]
0.002: (move walle floor table)  [0.001]
0.003: (move eva paper_trashbin organic_trashbin)  [0.001]
0.003: (pick book table walle wg2)  [0.001]
0.003: (drop bottle table walle wg1)  [0.001]
0.004: (drop banana organic_trashbin eva eg1)  [0.001]
0.004: (drop rotten_apple organic_trashbin eva eg2)  [0.001]
0.004: (move walle table fridge)  [0.001]
0.005: (pick newspaper fridge walle wg1)  [0.001]
0.006: (move walle fridge table)  [0.001]
0.007: (drop newspaper table walle wg1)  [0.001]
0.008: (move walle table shelving)  [0.001]
0.009: (drop book shelving walle wg2)  [0.001]
```

