(:durative-action social_move
  :parameters (?r - robot ?from ?to - waypoint ?room - room)
  :duration ( = ?duration 10)
  :condition (and
    (at start(waypoint_at ?from ?room))
    (at start(waypoint_at ?to ?room))
    (at start(robot_at_room ?r ?room))
    (at start(robot_at ?r ?from))
  )
  :effect (and
    (at start(not(robot_at ?r ?from)))
    (at end(robot_at ?r ?to))
    (at end(social_move_pred ?r ?to))
  )
)
