;;; ==========================================================================
;;; UAV Search — SACE Stage 2: Fault-injected variant (weather degraded)
;;; ==========================================================================
;;; Weather is NOT OK at sector_a. The planner should find NO plan that
;;; reaches sector_a — demonstrating that the ODD gating works.
;;; If a plan IS found, the hazard "UAV enters degraded weather" is reachable.

(define (problem uav-sace-weather-fault)
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
    ;; FAULT: weather NOT ok at sector_a
    (weather-ok sector_b)
    (comms-available)
  )

  (:goal (and
    (searched sector_a)
    (classified sector_a)
  ))
)
