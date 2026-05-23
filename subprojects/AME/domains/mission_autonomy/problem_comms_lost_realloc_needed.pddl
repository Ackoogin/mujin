;;; ==========================================================================
;;; Mission Autonomy — Comms lost + reallocation needed
;;; ==========================================================================
;;; Comms are down AND uav2 is unavailable. sector_b was assigned to uav2.
;;; reallocate-task requires comms, so reallocation is impossible.
;;;
;;; Expected: UNSOLVABLE — demonstrates that comms loss prevents dynamic
;;; task reallocation, exposing the need for pre-planned fallback
;;; assignments or autonomous reallocation protocols.

(define (problem mission-comms-lost-realloc-needed)
  (:domain mission-autonomy)

  (:objects
    uav1 uav2 - agent
    base - location
    sector_a sector_b - sector
  )

  (:init
    (at uav1 base)
    (at uav2 sector_b)

    (agent-available uav1)
    ;; FAULT: uav2 unavailable
    (agent-unavailable uav2)
    (sensor-operational uav1)
    (sensor-operational uav2)
    ;; FAULT: comms-available ABSENT

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
