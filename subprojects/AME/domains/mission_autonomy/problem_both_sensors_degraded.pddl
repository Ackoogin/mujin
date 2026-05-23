;;; ==========================================================================
;;; Mission Autonomy — Both sensors degraded
;;; ==========================================================================
;;; Both agents have sensor failures (sensor-degraded set, sensor-operational
;;; absent). search-degraded can still mark sectors as searched, but
;;; classify requires sensor-operational.
;;;
;;; Expected: UNSOLVABLE — classify goal cannot be met without working
;;; sensors. Demonstrates the boundary of degraded-mode capability.

(define (problem mission-both-sensors-degraded)
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
    ;; FAULT: both sensors degraded
    (sensor-degraded uav1)
    (sensor-degraded uav2)
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
