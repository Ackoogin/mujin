;;; ==========================================================================
;;; Vehicle Autonomy — Comms lost while airborne
;;; ==========================================================================
;;; UAV is airborne at wp1, comms link lost. Nav still works.
;;; Nominal fly requires comms, so must use emergency-return-nav.
;;;
;;; Expected: SOLVABLE via emergency-return-nav + emergency-land-at-base.

(define (problem vehicle-comms-lost-airborne)
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

    (nav-operational)
    (engines-operational)
    ;; FAULT: comms-available ABSENT
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
