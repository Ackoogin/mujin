;;; ==========================================================================
;;; UAV Search — SACE Stage 7: Out-of-context recovery
;;; ==========================================================================
;;; The UAV is at sector_a but comms have been lost and weather is degrading.
;;; Goal: reach a safe state (back at base). This tests whether safe-state
;;; reachability holds under degraded conditions.
;;;
;;; NOTE: With the baseline domain, move requires (comms-available), so
;;; this problem is UNSOLVABLE — revealing the need for a fallback
;;; move action that works without comms (Stage 5 design improvement).

(define (problem uav-sace-ooc-recovery)
  (:domain uav-search-sace)

  (:objects
    uav1 - robot
    base - location
    sector_a - sector
    sector_b - sector
  )

  (:init
    (at uav1 sector_a)
    (airspace-clear base)
    (airspace-clear sector_a)
    (airspace-clear sector_b)
    (weather-ok base)
    ;; Weather degrading at current location
    (weather-ok sector_b)
    ;; Comms lost
  )

  (:goal (at uav1 base))
)
