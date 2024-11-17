[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-24ddc0f5d75046c5622901739e7c5dd533143b0c8e959d652212380cedb1ea36.svg)](https://classroom.github.com/a/-Bce75SB)
# P3 - Depósito 2.0

En esta práctica vamos a partir de la solución de la práctica anterior basada en acciones durativas. Crea un fichero llamado [deposit_domain.pddl](deposit_domain.pddl) para la definición del dominio y otro llamado [deposit_problem.pddl](deposit_problem.pddl) para la definición del problema, e incluye tu solución de la última práctica.

## Ejercicio 1
Si aún no has hecho el ejercicio extra de la P2, modifica la duración de la acción `move` para que dependa de la distancia entre el origen y el destino. Además, añade un predicado para saber si dos ubicaciones están conectadas. La acción `move` sólo se podrá realizar si el origen y el destino están conectados.

## Ejercicio 2
Modifica el [problema](deposit_problem.pddl) para que el `goal` no esté expresado para cada objeto de forma individual, sino para todos a la vez (usando forall).
Prueba a generar el plan con POPF y con OPTIC y comenta las diferencias.

``` pddl
(:goal (and
    (item_at bottle big_container)                                              (:goal (and
    (item_at rotten_apple big_container)                                               (forall (?obj -item ) 
    (item_at newspaper big_container)             ------------------>                  (item_at ?obj big_container)
    (item_at book big_container)                                                       )
    (item_at banana big_container)                                                     )
                                                                                )

  )
)                                                                                                                                                       
``` 
POPF no soprta la instruccion forall en las sentencias de goal y da segmentation, en cambio optic acepta perfetamente el cambio y ejecuta de  la misma manera que en el apartado anterior.

## Ejercicio 3
No nos terminamos de fiar de los planificadores, ya que a veces nos dan planes en los que el robot no termina de llenar su contenedor antes de volver al depósito.
Por este motivo, vamos a modificar el dominio para que sea más restrictivo, y no vamos a permitir al robot volver a su base hasta que no haya recogido todos los objetos.

Implementa una acción adicional para que el robot vuelva a su base, donde está el depósito. Esta acción sólo se podrá realizar una vez que todos los objetos de tipo `item` hayan sido recogidos por el robot.
Explica cómo has definido los requisitos para que se cumpla la restricción propuesta:

``` pddl
(:durative-action move
  :parameters (?r - robot ?from ?to - location ?g - gripper)
  :duration (= ?duration (distance ?from ?to))
  :condition
    (and 
      (at start (robot_at ?r ?from))
      (over all (gripper_free ?r ?g))  
      ;(over all (or (connected ?from ?to) (connected ?to ?from ))) ;or en precondicion no falla pero no deja funcionar al planificador
      (over all (connected ?from ?to))
      ;(over all (connected ?to ?from ))
     ** (over all (not_container ?to))**
    )
  :effect
    (and
      (at start (not (robot_at ?r ?from)))
      (at end (robot_at ?r ?to))
    )
)

(:durative-action goto_base
:parameters (?r - robot ?l - location ?c - container ?g - gripper)
:duration (= ?duration 1)
:condition
  (and 
    (at start(robot_at ?r ?l))
    **(over all (connected ?l Big_Container))**
    **(at start(forall(?obj - item)
      (object_in_container ?obj ?r ?c)))**
  )
:effect
  (and 
    (at start(not (robot_at ?r ?l)))
    (at end(robot_at ?r Big_Container))
  )
)

```

Para obligar al planificador a realizar la carga de todos los objetos, tendremos que por una parte asegurarnos que el movimiento a traves de move nunca se realiza hacia el contenedor de forma que si quisiera desacargar tendria que utilizar la accion goto_base que tiene como prerequisio que todos los objetos del escenario esten cargados en el contendor propio del robot. 

*Nota:* Esta acción tiene el objetivo de evitar que el robot vaya al depósito a descargar (`unload`) hasta que no haya recogido toda la basura. Si se te ocurre una solución alternativa, impleméntala y explica su funcionamiento.

*Pistas:*
* Es posible definir el depósito principal como una constante.
* Si añadimos la restricción de que todos los objetos deben ser cargados en el robot antes de ir al depósito, es posible que haya que aumentar la capacidad máxima de carga del robot para que el problema tenga solución.



## Ejercicio extra [*Opcional*]
Implementa en [UPF](https://unified-planning.readthedocs.io/en/latest/) alguno de los dominios/problemas usados en las últimas 3 prácticas y compara los resultados.
