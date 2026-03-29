(define (domain multi-uav-search)
  (:requirements :strips :typing)

  (:types
    location - object
    sector - location
    agent - object
  )

  (:predicates
    (at ?a - agent ?l - location)
    (searched ?s - sector)
    (classified ?s - sector)
  )

  (:action move
    :parameters (?a - agent ?from - location ?to - location)
    :precondition (at ?a ?from)
    :effect (and (at ?a ?to) (not (at ?a ?from))))

  (:action search
    :parameters (?a - agent ?s - sector)
    :precondition (at ?a ?s)
    :effect (searched ?s))

  (:action classify
    :parameters (?a - agent ?s - sector)
    :precondition (and (at ?a ?s) (searched ?s))
    :effect (classified ?s))
)
