[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-24ddc0f5d75046c5622901739e7c5dd533143b0c8e959d652212380cedb1ea36.svg)](https://classroom.github.com/a/VFwGcsAz)
# Trabajo Final PDDL

El trabajo final del bloque de PDDL consistirá en una comparación de distintos planificadores, analizando qué features de PDDL están soportadas por cada uno.

*Nota:* Este trabajo se realizará por grupos de 4 personas. Todas las personas del grupo debéis aceptar la tarea en Github Classroom y entrar en el repositorio creado automáticamente para el grupo.

## Planificadores

Los planificadores a comparar serán POPF, OPTIC y uno más a elección del grupo. Indicad aquí cuál es el planificador adicional escogido, cuál es la web oficial, dónde se encuentra su documentación y cuáles son los pasos para poder instalarlo y ejecutarlo:

Hemos elegido YAHSP como el planificador sobre el que haremos este trabajo. La página web oficial de este es: http://v.vidal.free.fr/onera/#yahsp, aquí se encuentra tanto una breve descripción acerca del planer, los papers que se han escrito sobre este y los links de descarga. Para poder instalarlo sólo hay que descargarlo con uno de estos links y descomprimirlo, una vez hecho esto para poder ejecutarlo, hay que darle permisos de ejecución al fichero. Al ejecutarlo sin argumentos aparecerán instrucciones sobre cómo ejecutarlo correctamente, las cuales se resumen en: ./yahsp dominio problema [algoritmo de búsqueda].


## Estudio de compatibilidad con features de PDDL
A continuación, seleccionad un subset de features que os parezcan interesantes del lenguaje. Por ejemplo: `durative-actions`, `existential-preconditions`, `negative-precontitions`, etc.). Después, verificad si están soportadas por cada uno de los planificadores.

Para demostrar el soporte de cada feature, es necesario hacer dos cosas:

1. Comprobar en la documentación (si la hubiese) si el planificador soporta dicha feature.
2. Crear un ejemplo mínimo, con un dominio y un problema sencillos en el que se utilice dicha feature para demostrarlo, comprobando el comportamiento de cada uno de los planificadores.

Tras hacer estos análisis hemos llegado a la conclusión de que yahsp es el planificador más limitado, mientras que optic es el que tiene más funciones diponibles. Mientras que yahsp no soporta fluents, durative actions o foralls, optic puede usar todas estas funciones. Por otro lado popf se acerca bastante a optic, no siendo capaz de usar forall en sus planes. Además, los tres planificadores soportan tipos y distintos algoritmos de búsqueda.


Finalmente, generad una tabla en la que se resuma la compatibilidad de cada planificador, con las features analizadas por filas y los planificadores por columnas. Podéis encontrar información de cómo generar tablas en formato Markdown de Github [aquí](https://docs.github.com/es/get-started/writing-on-github/working-with-advanced-formatting/organizing-information-with-tables).

|                           	|        POPF        	|        OPTIC       	|        YAHSP       	|
|:-------------------------:	|:------------------:	|:------------------:	|:------------------:	|
|           Forall          	|         :x:        	| :white_check_mark: 	|         :x:        	|
|      Durative_actions     	| :white_check_mark: 	| :white_check_mark: 	|         :x:        	|
|          Fluents          	| :white_check_mark: 	| :white_check_mark: 	|         :x:        	|
|           Types           	| :white_check_mark: 	| :white_check_mark: 	| :white_check_mark: 	|
|   Various Search Engines  	| :white_check_mark: 	| :white_check_mark: 	| :white_check_mark: 	|
| Existencial-preconditions 	|         :x:        	|         :x:        	|         :x:        	|
|   Negative-preconditions  	|         :x:        	|         :x:        	|         :x:        	|

## Entrega del trabajo
El trabajo se presentará en clase el próximo lunes 26 de febrero. Además, en este repositorio deberéis añadir los ficheros PDDL para los ejemplos generados, debidamente referenciados en el README.

Es recomendable preparar diapositivas para la presentación, las cuales deberéis añadir también al repositorio.
