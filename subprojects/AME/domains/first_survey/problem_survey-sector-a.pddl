(define (problem first-survey-1)
  (:domain first-survey)

  (:objects
    uav1 - robot
    base - location
    sector_a - sector
    sector_b - sector
  )

  (:init
    (at uav1 base)
    (on-ground uav1)
    (comms-available)
  )

  (:goal (and
    (reported sector_a)
    (at uav1 base)
    (on-ground uav1)
  ))
)
