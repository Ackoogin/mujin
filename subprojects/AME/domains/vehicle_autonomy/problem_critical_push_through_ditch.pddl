;;; ==========================================================================
;;; Vehicle Autonomy — Critical mission: push through degradation to ditch
;;; ==========================================================================
;;; Critical mission, airborne at base, nav + comms failed. A non-critical
;;; mission would abort or emergency-return. Critical priority enables the
;;; vehicle to push through using GPS-only flight to reach the mission
;;; area, complete the mission, then fly to the designated ditch zone and
;;; hard-land (controlled destruction).
;;;
;;; Expected: SOLVABLE via:
;;;   fly-critical-gps(home→wp1) → execute-mission-critical
;;;     → fly-to-ditch-gps(wp1→dz1) → hard-land-ditch (4 steps)

(define (problem vehicle-critical-push-through-ditch)
  (:domain vehicle-autonomy)

  (:objects
    uav1 - robot
    home - base
    wp1 - waypoint
    dz1 - waypoint
  )

  (:init
    (at uav1 home)
    (airborne uav1)
    (preflight-done uav1)

    (mission-priority-critical)

    (mission-objective wp1)

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
