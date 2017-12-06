# Main controller that has a RobotWorld, takes in paths from either
# SimulatedPathGenerator or gets next path from PathGenerator (which works
# on whats currently drawn and the desired image reconstruction), runs
# a TimelineGenerator to generate the timeline of joint variables, and
# eventually will send this information to actual robot arm to execute
