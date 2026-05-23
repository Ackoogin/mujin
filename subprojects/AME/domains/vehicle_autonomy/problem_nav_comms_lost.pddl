;;; ==========================================================================
;;; Vehicle Autonomy — Nav + comms lost (GPS available)
;;; ==========================================================================
;;; Both nav and comms failed while airborne. GPS still works.
;;; Tests the middle tier of the redundancy ladder.
;;;
;;; Expected: SOLVABLE via emergency-return-gps + emergency-land-at-base.

(define (problem vehicle-nav-comms-lost)
  (:domain vehicle-autonomy)

  (:objects
    uav1 - robot
    home - base
    wp1 - waypoint
  )

  (:init
    (at uav1 wp1)
    (airborne uav1)
    (preflight-done uav1)

    ;; FAULTS: nav-operational ABSENT, comms-available ABSENT
    (engines-operational)
    (gps-available)
    (inertial-nav-available)

    (airspace-clear home)
    (airspace-clear wp1)
    (weather-ok home)
    (weather-ok wp1)
  )

  (:goal (and
    (safe-state uav1)
    (landed-safe uav1)
    (at uav1 home)
  ))
)
