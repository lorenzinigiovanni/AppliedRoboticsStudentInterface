(define (domain pursuer-evader)
    (:requirements :adl :derived-predicates :action-costs)
    (:types
        evader pursuer - robot
        gate waypoint - location
    )
    (:constants
        r1 - pursuer
        r2 - evader
    )
    (:predicates
        (at ?r - robot ?l - location)
        (near ?l1 ?l2 - location)
        (evaded ?e - evader)
        (caught ?p - pursuer)
        (pursuing)
        (evading)
    )
    (:functions
        (total-cost)
        (evader-cost ?l1 - location)
        (distance ?l1 ?l2 - location)
    )
    (:action move
        :parameters (?r - robot ?from ?to - location)
        :precondition (and
            (or
                (and
                    (= ?r r1)
                    (pursuing)
                )
                (and
                    (= ?r r2)
                    (evading)
                )
            )
            (or
                (near ?from ?to)
                (near ?to ?from)
            )
            (at ?r ?from)
        )
        :effect (and
            (at ?r ?to)
            (not (at ?r ?from))
            (increase (total-cost) (distance ?from ?to))
        )
    )
    (:derived
        (evaded ?e - evader)
        (and
            (exists
                (?g - gate)
                (at ?e ?g)
            )
        )
    )
    (:derived
        (caught ?p - pursuer)
        (and
            (exists
                (?l - location)
                (and
                    (at r1 ?l)
                    (<= (total-cost) (evader-cost ?l))
                )
            )
        )
    )
)
