; problem to debug pddl
(define (problem pursuer-prob)
    (:domain pursuer-escaper)
    (:objects
        l0 l1 l2 l3 l4 - waypoint
        l5 - gate
    )
    (:init
        (at r1 l0)
        (at r2 l3)

        ;(excaping)
        (pursuing)

        (= (total-cost) 0)

        (near l0 l1)
        (near l1 l2)
        (near l2 l3)
        (near l3 l4)
	    (near l3 l5)
        (near l4 l5)

        (= (distance l0 l1) 4)
        (= (distance l1 l0) 4)
        (= (distance l1 l2) 7)
        (= (distance l2 l1) 7)
        (= (distance l2 l3) 5)
        (= (distance l3 l2) 5)
        (= (distance l3 l4) 20)
        (= (distance l4 l3) 20)
        (= (distance l4 l5) 13)
        (= (distance l5 l4) 13)
	    (= (distance l3 l5) 60)
        (= (distance l5 l3) 60)

    )
    (:goal
        ;(escaped r2)
        (caught r1 r2)
        )
    (:metric minimize(total-cost)
    )
)
