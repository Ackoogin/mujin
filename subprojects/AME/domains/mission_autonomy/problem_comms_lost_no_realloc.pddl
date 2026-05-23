;;; ==========================================================================
;;; Mission Autonomy — Comms lost: cannot reallocate tasks
;;; ==========================================================================
;;; Comms are down. reallocate-task and assign-task both require comms.
;;; Each agent can only complete tasks already assigned to it.
;;; uav1 has sector_a assigned, uav2 has sector_b — both available.
;;;
;;; Expected: SOLVABLE — each agent completes its pre-assigned task
;;; without reallocation. Demonstrates that pre-assigned tasks are
;;; resilient to comms loss.

(define (problem mission-comms-lost-no-realloc)
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
    ;; FAULT: comms-available ABSENT

    ;; Tasks pre-assigned before comms loss
    (task-assigned sector_a uav1)
    (task-assigned sector_b uav2)
  )

  (:goal (and
    (searched sector_a)
    (classified sector_a)
    (searched sector_b)
    (classified sector_b)
  ))
)
