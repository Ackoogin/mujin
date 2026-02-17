(define (problem uav-search-1)
  (:domain uav-search)

  (:objects
    uav1 - robot
    base - location
    sector_a - sector
    sector_b - sector
  )

  (:init
    (at uav1 base)
  )

  (:goal (and
    (searched sector_a)
    (classified sector_a)
  ))
)
