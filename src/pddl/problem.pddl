; problem to debug pddl
(define (problem pursuer-prob)
    (:domain pursuer)
    (:objects
        l0 l1 l2 l3 l4 l5 - location
        r1 r2 - robot
    )
    (:init
        (at r1 l0)
        (at r2 l4)

        (= (total-cost) 0)

        (near l0 l1)
        (near l1 l2)
        (near l2 l3)
        (near l3 l4)
        (near l4 l5)

        (= (distance l0 l1) 4)
        (= (distance l1 l2) 7)
        (= (distance l2 l3) 5)
        (= (distance l3 l4) 2)
        (= (distance l4 l5) 13)

        (is-gate l5)
    )
    (:goal
        (or
            (escaped r2)
            (caught r1 r2)
        )
    )
    ; (:metric minimize
    ;     (total-cost)
    ; )
)
