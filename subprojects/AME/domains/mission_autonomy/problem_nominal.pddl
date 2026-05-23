;;; ==========================================================================
;;; Mission Autonomy — Nominal: two agents, two sectors
;;; ==========================================================================
;;; Both agents available with working sensors. Tasks pre-assigned.
;;; No contingency actions needed.
;;;
;;; Expected: SOLVABLE — each agent completes its assigned sector.

(define (problem mission-nominal)
  (:domain mission-autonomy)

  (:objects
    uav1 uav2 - agent
    base - location
    sector_a sector_b - sector
  )

  (:init
    (at uav1 base)
    (at uav2 base)
    (agent-available uav1)
    (agent-available uav2)
    (sensor-operational uav1)
    (sensor-operational uav2)
    (comms-available)

    (task-assigned sector_a uav1)
    (task-assigned sector_b uav2)
  )

  (:goal (and
    (searched sector_a)
    (searched sector_b)
    (classified sector_a)
    (classified sector_b)
  ))
)
