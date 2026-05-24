;;; ==========================================================================
;;; Vehicle Autonomy — Critical mission: nav + comms failed, GPS ditch
;;; ==========================================================================
;;; Critical mission, airborne at mission waypoint, nav and comms lost.
;;; GPS and inertial still available. Vehicle completes mission under
;;; critical rules (no comms needed), navigates to ditch zone via GPS,
;;; and performs a controlled hard landing (deliberate destruction).
;;;
;;; Expected: SOLVABLE via:
;;;   execute-mission-critical → fly-to-ditch-gps(wp1→dz1)
;;;     → hard-land-ditch (3 steps)

(define (problem vehicle-critical-nav-fail-ditch)
  (:domain vehicle-autonomy)

  (:objects
    uav1 - robot
    home - base
    wp1 - waypoint
    dz1 - waypoint
  )

  (:init
    (at uav1 wp1)
    (airborne uav1)
    (preflight-done uav1)

    (mission-priority-critical)

    ;; FAULTS: nav-operational ABSENT, comms-available ABSENT
    (engines-operational)
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
    (vehicle-ditched uav1)
    (safe-state uav1)
  ))
)
