(define (domain pursuer-escaper)
    (:requirements :adl :derived-predicates :action-costs)
    (:types
        escaper pursuer - robot
        gate waypoint - location
    )
    (:constants
        r1 - pursuer
        r2 - escaper
    )
    (:predicates
        (at ?r - robot ?l - location)
        (near ?l1 ?l2 - location)
        (escaped ?e - escaper)
        (caught ?p - pursuer)
        (pursuing)
        (escaping)
    )
    (:functions
        (total-cost)
        (escaper-cost ?l1)
        (distance ?l1 ?l2)
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
                    (escaping)
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
        (escaped ?e - escaper)
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
                    (<= (total-cost) (escaper-cost ?l))
                )
            )
        )
    )
)
