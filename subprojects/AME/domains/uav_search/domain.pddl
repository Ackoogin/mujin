(define (domain uav-search)
  (:requirements :strips :typing)

  (:types
    location - object
    sector - location
    robot - object
  )

  (:predicates
    (at ?r - robot ?l - location)
    (searched ?s - sector)
    (classified ?s - sector)
  )

  (:action move
    :parameters (?r - robot ?from - location ?to - location)
    :precondition (at ?r ?from)
    :effect (and
      (at ?r ?to)
      (not (at ?r ?from))
    )
  )

  (:action search
    :parameters (?r - robot ?s - sector)
    :precondition (at ?r ?s)
    :effect (searched ?s)
  )

  (:action classify
    :parameters (?r - robot ?s - sector)
    :precondition (and
      (at ?r ?s)
      (searched ?s)
    )
    :effect (classified ?s)
  )
)
