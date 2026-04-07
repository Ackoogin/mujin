;;; ==========================================================================
;;; UAV Search — SACE Stage 1: Nominal operating scenario
;;; ==========================================================================
;;; All ODD conditions satisfied. The planner should find a valid plan.

(define (problem uav-sace-nominal)
  (:domain uav-search-sace)

  (:objects
    uav1 - robot
    base - location
    sector_a - sector
    sector_b - sector
  )

  (:init
    (at uav1 base)
    ;; ODD conditions — all clear
    (airspace-clear base)
    (airspace-clear sector_a)
    (airspace-clear sector_b)
    (weather-ok base)
    (weather-ok sector_a)
    (weather-ok sector_b)
    (comms-available)
  )

  (:goal (and
    (searched sector_a)
    (classified sector_a)
  ))
)
