;;; ==========================================================================
;;; Vehicle Autonomy — Total navigation loss (inertial only)
;;; ==========================================================================
;;; Nav, GPS, comms all lost. Only inertial navigation remains.
;;; Bottom of the redundancy ladder before last-resort emergency-land.
;;;
;;; Expected: SOLVABLE via emergency-return-inertial + emergency-land-at-base.

(define (problem vehicle-total-nav-loss)
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

    ;; FAULTS: nav, GPS, comms all ABSENT
    (engines-operational)
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
