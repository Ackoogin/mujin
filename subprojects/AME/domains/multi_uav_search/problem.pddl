(define (problem multi-uav-recon)
  (:domain multi-uav-search)

  (:objects
    uav1 uav2 - agent
    base - location
    sector_a sector_b sector_c sector_d - sector
  )

  (:init
    (at uav1 base)
    (at uav2 base)
  )

  (:goal (and
    (searched sector_a)
    (searched sector_b)
    (searched sector_c)
    (searched sector_d)
    (classified sector_a)
    (classified sector_b)
    (classified sector_c)
    (classified sector_d)
  ))
)
