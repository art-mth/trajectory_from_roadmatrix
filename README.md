# trajectory_from_roadmatrix
Given a matrix representation of the road creates a trajectory that tries to optimize on certain criteria.

Current criteria for trajectory choice include:
- Avoiding obstacles
- Staying close to the "perfect" trajectory (middle of right lane)

This also means trying to wait close to the perfect trajectory in uncertain situations (e.g. Even though the road is not blocked there is not enough space to overtake).

## Data channels
- ROADMATRIX(street_environment::RoadMatrix)
- TRAJECTORY(street_environment::Trajectory)

## Config
- carWidthMeter: The width of the car in meter.

Clearance values are used to set the clearance that the car should adhere to when choosing a trajectory. For example it makes sense to limit how close the car should drive to obstacles. This is especially important when overtaking an obstacle. By setting LeftFront clearance values higher the car is more cautious when overtaking on the left and would rather wait on the right than risk hitting an obstacle. By setting RightBack configs higher the car will wait until moving back to the right lane after overtaking which gives a smoother trajectory.

- obstacleClearanceLeftFrontMeter: LeftFront is everything that is in front of the vehicle left of the perfect trajectory.
- obstacleClearanceRightFrontMeter: RightFront is everything that is on or right of the perfect trajectory.
- obstacleClearanceLeftBackMeter: Same as above to the back
- obstacleClearanceRightBackMeter: Same as above to the back

