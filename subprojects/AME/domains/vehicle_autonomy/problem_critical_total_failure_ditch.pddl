;;; ==========================================================================
;;; Vehicle Autonomy — Critical mission: total failure, terminal ditch
;;; ==========================================================================
;;; Critical mission, airborne at mission waypoint. ALL systems down —
;;; no nav, no GPS, no comms, no engines, no inertial. Vehicle completes
;;; mission under critical rules (execute-mission-critical has no system
;;; requirements) then performs terminal ditch at current position.
;;; Cannot reach the designated ditch zone — self-destructs in place.
;;;
;;; Expected: SOLVABLE via:
;;;   execute-mission-critical → terminal-ditch (2 steps)

(define (problem vehicle-critical-total-failure-ditch)
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

    (mission-objective wp1)

    ;; ALL SYSTEMS DOWN

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
