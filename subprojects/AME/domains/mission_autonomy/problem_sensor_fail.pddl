;;; ==========================================================================
;;; Mission Autonomy — Sensor failure: uav1 sensor down, reallocate
;;; ==========================================================================
;;; uav1 is at sector_a but its sensor has failed. It is still
;;; agent-available but has sensor-degraded set. uav2 is available with
;;; working sensors.
;;;
;;; Recovery path:
;;;   withdraw-agent(uav1) -> mark-task-failed(sector_a, uav1)
;;;   -> reallocate-task(sector_a, uav2) -> move uav2 -> search -> classify
;;;
;;; Expected: SOLVABLE via withdraw + mark-task-failed + reallocate.

(define (problem mission-sensor-fail)
  (:domain mission-autonomy)

  (:objects
    uav1 uav2 - agent
    base - location
    sector_a sector_b - sector
  )

  (:init
    (at uav1 sector_a)
    (at uav2 base)

    (agent-available uav1)
    (agent-available uav2)
    ;; FAULT: uav1 sensor degraded
    (sensor-degraded uav1)
    (sensor-operational uav2)
    (comms-available)

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
