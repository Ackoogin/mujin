;;; ==========================================================================
;;; UAV Search — SACE Stage 2: Fault-injected variant (comms lost)
;;; ==========================================================================
;;; C2 link is down. The mission goal should be unreachable, but the
;;; safe-state action (loiter-in-place) should still be applicable.

(define (problem uav-sace-comms-lost)
  (:domain uav-search-sace)

  (:objects
    uav1 - robot
    base - location
    sector_a - sector
    sector_b - sector
  )

  (:init
    (at uav1 base)
    (airspace-clear base)
    (airspace-clear sector_a)
    (airspace-clear sector_b)
    (weather-ok base)
    (weather-ok sector_a)
    (weather-ok sector_b)
    ;; FAULT: comms NOT available
  )

  (:goal (and
    (searched sector_a)
    (classified sector_a)
  ))
)
