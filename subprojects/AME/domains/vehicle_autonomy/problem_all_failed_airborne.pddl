;;; ==========================================================================
;;; Vehicle Autonomy — Catastrophic: all systems failed while airborne
;;; ==========================================================================
;;; Every system is down. emergency-land is the absolute last resort —
;;; its only precondition is (airborne). Proves the domain always has
;;; a path to safe-state even in the worst case.
;;;
;;; Expected: SOLVABLE via emergency-land (1 step).

(define (problem vehicle-all-failed-airborne)
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

    ;; ALL SYSTEMS DOWN

    (airspace-clear home)
    (airspace-clear wp1)
    (weather-ok home)
    (weather-ok wp1)
  )

  (:goal (and
    (safe-state uav1)
    (landed-safe uav1)
  ))
)
