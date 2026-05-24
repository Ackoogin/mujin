;;; ==========================================================================
;;; Vehicle Autonomy — Critical mission: severe degradation, inertial ditch
;;; ==========================================================================
;;; Critical mission, airborne at mission waypoint. Nav, GPS, and comms
;;; all lost — only inertial navigation and engines remain. Vehicle
;;; completes mission under critical rules, navigates to ditch zone using
;;; inertial-only nav, and performs controlled destruction.
;;;
;;; Expected: SOLVABLE via:
;;;   execute-mission-critical → fly-to-ditch-inertial(wp1→dz1)
;;;     → hard-land-ditch (3 steps)

(define (problem vehicle-critical-severe-degrade-ditch)
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

    ;; FAULTS: nav-operational, gps-available, comms-available all ABSENT
    (engines-operational)
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
