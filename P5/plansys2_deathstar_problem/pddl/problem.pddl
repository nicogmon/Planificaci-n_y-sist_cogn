(define (problem deathstar-problem)
(:domain deathstar)
    (:objects
        star - deathstar
        endor coruscant alderaan dantooine hoth yavin4 - planet
    )
    
    (:init
        (star_at star alderaan)

        (connected coruscant endor)
        (connected endor coruscant)
        (connected coruscant alderaan)
        (connected alderaan coruscant)
        (connected coruscant dantooine)
        (connected dantooine coruscant)
        (connected endor hoth)
        (connected hoth endor)
        (connected endor yavin4)
        (connected yavin4 endor)
    )
    
    (:goal
    (and
        (destroyed yavin4)
        (destroyed hoth)
    )
    )
) ; End Problem
