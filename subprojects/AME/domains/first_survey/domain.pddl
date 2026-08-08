(define (domain first-survey)
  (:requirements :strips :typing)

  (:types
    location - object
    sector - location
    robot - object
  )

  (:predicates
    (at ?r - robot ?l - location)
    (on-ground ?r - robot)
    (airborne ?r - robot)
    (searched ?s - sector)
    (reported ?s - sector)
    (comms-available)
  )

  (:action take-off
    :parameters (?r - robot ?l - location)
    :precondition (and
      (at ?r ?l)
      (on-ground ?r)
    )
    :effect (and
      (airborne ?r)
      (not (on-ground ?r))
    )
  )

  (:action fly
    :parameters (?r - robot ?from - location ?to - location)
    :precondition (and
      (at ?r ?from)
      (airborne ?r)
    )
    :effect (and
      (at ?r ?to)
      (not (at ?r ?from))
    )
  )

  (:action survey
    :parameters (?r - robot ?s - sector)
    :precondition (and
      (at ?r ?s)
      (airborne ?r)
    )
    :effect (searched ?s)
  )

  (:action report
    :parameters (?r - robot ?s - sector)
    :precondition (and
      (at ?r ?s)
      (searched ?s)
      (comms-available)
    )
    :effect (reported ?s)
  )

  (:action land
    :parameters (?r - robot ?l - location)
    :precondition (and
      (at ?r ?l)
      (airborne ?r)
    )
    :effect (and
      (on-ground ?r)
      (not (airborne ?r))
    )
  )
)
