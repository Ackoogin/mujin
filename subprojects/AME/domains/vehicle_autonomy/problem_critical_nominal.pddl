;;; ==========================================================================
;;; Vehicle Autonomy — Critical mission: nominal (all systems operational)
;;; ==========================================================================
;;; Mission priority is critical but all systems are operational.
;;; Validates that critical priority does not force ditching when the
;;; vehicle can complete the mission and return home normally.
;;;
;;; Expected: SOLVABLE via normal nominal path.
;;;   preflight -> takeoff -> fly -> execute-mission -> fly-back -> land
;;; Ditch zone exists but is not used.

(define (problem vehicle-critical-nominal)
  (:domain vehicle-autonomy)

  (:objects
    uav1 - robot
    home - base
    wp1 - waypoint
    dz1 - waypoint
  )

  (:init
    (at uav1 home)
    (on-ground uav1)

    (mission-priority-critical)

    (nav-operational)
    (engines-operational)
    (comms-available)
    (gps-available)
    (inertial-nav-available)

    (airspace-clear home)
    (airspace-clear wp1)
    (airspace-clear dz1)
    (weather-ok home)
    (weather-ok wp1)
    (weather-ok dz1)

    (ditch-zone-designated dz1)
  )

  (:goal (and
    (mission-complete uav1)
    (landed-safe uav1)
    (at uav1 home)
  ))
)
