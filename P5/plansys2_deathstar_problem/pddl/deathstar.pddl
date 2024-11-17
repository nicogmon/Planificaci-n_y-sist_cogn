(define (domain deathstar)
(:requirements :strips :typing :durative-actions)

; Types
(:types
    deathstar
    planet
)

; Predicates
(:predicates
    (star_at ?d - deathstar ?p - planet)
    (connected ?p1 ?p2 - planet)
    (destroyed ?p - planet)
)

; Actions
(:durative-action travel
    :parameters (?d - deathstar ?p1 ?p2 - planet)
    :duration ( = ?duration 10)
    :condition (and
        (over all (connected ?p1 ?p2))
        (at start (star_at ?d ?p1))
        )
    :effect (and
        (at start (not(star_at ?d ?p1)))
        (at end (star_at ?d ?p2))
    )
)

(:durative-action destroy_planet
    :parameters (?d - deathstar ?p - planet)
    :duration ( = ?duration 5)
    :condition (and
        (over all (star_at ?d ?p))
       )
    :effect (and
        (at end (destroyed ?p))
    )
)

); End Domain
