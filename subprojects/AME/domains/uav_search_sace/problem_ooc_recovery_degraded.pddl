;;; ==========================================================================
;;; UAV Search — SACE Stage 5/7: Recovery with degraded-mode design
;;; ==========================================================================
;;; Same scenario as problem_ooc_recovery.pddl but using the degraded domain.
;;; With emergency-return-inertial available, the planner should now find a
;;; plan to return to base even without comms or GPS — proving the Stage 5
;;; design improvement resolves the Stage 7 gap.

(define (problem uav-sace-ooc-recovery-degraded)
  (:domain uav-search-sace-degraded)

  (:objects
    uav1 - robot
    base - location
    sector_a - sector
    sector_b - sector
  )

  (:init
    (at uav1 sector_a)
    (airspace-clear base)
    (airspace-clear sector_a)
    (airspace-clear sector_b)
    (weather-ok base)
    (weather-ok sector_b)
    ;; Comms lost, GPS denied — worst case
    (inertial-nav-available)
  )

  (:goal (at uav1 base))
)
