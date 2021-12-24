(define (domain pursuer)
    (:requirements :strips :typing :disjunctive-preconditions :action-costs)
    (:types
        robot location
    )
    (:predicates
        (at ?r - robot ?l - location)
        (near ?l1 ?l2 - location)
        (is-gate ?l - location)
        (escaped ?r - robot)
        (caught ?r1 ?r2 - robot)
    )
    (:functions
        (total-cost)
        (distance ?l1 ?l2)
    )
    (:action move
        :parameters (?r - robot ?from ?to - location)
        :precondition (and
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
            (increase (total-cost) (distance ?to ?from))
        )
    )
    (:action reach-gatess
        :parameters (?r - robot ?l - location)
        :precondition (and
            (is-gate ?l)
            (at ?r ?l)
        )
        :effect (and
            (escaped ?r)
        )
    )
    (:action reach-evader
        :parameters (?r1 ?r2 - robot ?l - location)
        :precondition (and
            (at ?r1 ?l)
            (at ?r2 ?l)
        )
        :effect (and
            (caught ?r1 ?r2)
            (not (escaped ?r2))
        )
    )
)
