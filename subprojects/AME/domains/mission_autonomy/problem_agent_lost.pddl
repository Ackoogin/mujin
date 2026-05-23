;;; ==========================================================================
;;; Mission Autonomy — Agent lost: uav2 unavailable, tasks reallocated
;;; ==========================================================================
;;; uav2 became unavailable (vehicle fault, RTB, etc.) after being assigned
;;; sectors c and d. Those tasks must be reallocated to uav1.
;;;
;;; Expected: SOLVABLE — mark-task-failed for c,d then reallocate to uav1,
;;; uav1 completes all four sectors.

(define (problem mission-agent-lost)
  (:domain mission-autonomy)

  (:objects
    uav1 uav2 - agent
    base - location
    sector_a sector_b sector_c sector_d - sector
  )

  (:init
    (at uav1 base)
    (at uav2 sector_c)

    (agent-available uav1)
    ;; FAULT: uav2 is unavailable
    (agent-unavailable uav2)
    (sensor-operational uav1)
    (sensor-operational uav2)
    (comms-available)

    ;; uav1 was assigned a,b — already completed a
    (task-assigned sector_a uav1)
    (task-assigned sector_b uav1)
    (searched sector_a)
    (classified sector_a)

    ;; uav2 was assigned c,d — but is now unavailable
    (task-assigned sector_c uav2)
    (task-assigned sector_d uav2)
  )

  (:goal (and
    (searched sector_a)
    (searched sector_b)
    (searched sector_c)
    (searched sector_d)
    (classified sector_a)
    (classified sector_b)
    (classified sector_c)
    (classified sector_d)
  ))
)
