;;; ==========================================================================
;;; Vehicle Autonomy — Nominal: full flight lifecycle
;;; ==========================================================================
;;; All systems operational. Planner finds:
;;;   preflight -> takeoff -> fly -> execute-mission -> fly-back -> land
;;;
;;; Expected: SOLVABLE, no contingency actions used.

(define (problem vehicle-nominal)
  (:domain vehicle-autonomy)

  (:objects
    uav1 - robot
    home - base
    wp1 - waypoint
  )

  (:init
    (at uav1 home)
    (on-ground uav1)

    (nav-operational)
    (engines-operational)
    (comms-available)
    (gps-available)
    (inertial-nav-available)

    (airspace-clear home)
    (airspace-clear wp1)
    (weather-ok home)
    (weather-ok wp1)
  )

  (:goal (and
    (mission-complete uav1)
    (landed-safe uav1)
    (at uav1 home)
  ))
)
