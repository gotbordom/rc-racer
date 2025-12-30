# RoboMagellan 2024 / 2025

Note that much of what is documented here comes from their webpage which can be found here: [RoboMagellan](https://robogames.net/rules/magellan.php)

## Objective
The Robo-Magellan is a robotics competition emphasizing autonomous navigation and obstacle avoidance over varied, outdoor terrain. Robots have three opportunities to navigate from a starting point to an ending point and are scored on time required to complete the course with opportunities to lower the score based on contacting intermediate points. These rules are from the Seattle Robotics Society's Robo-Magellan event.

### This means the car will need to be able to:
- Create a 2D weighted map of the course where the weight is likely elevation of the obstacle
- Add and remove obstacles as needed
- Detect markers, and add them to the map
- Detect the start / finish lines and add them to the map
- Path planning to find ideal path through all markers to finish line
  - Initially just find all markers and finish line.
  - Second learn to drive that path everytime.
  - Third start working towards a score weight where the controls _could_ drop markers to save time if it would create a higher score.

## Course Design
The course can, and will vary quite a bit between each year. 

### Obstacles and materials
The materials could be anything including, but not limited to:
- pavement
- dirt
- small rocks
- grass
- hills
- gullies
- trees
- curbs
- weeds

### Car Starting Point
- Starting point: Random starting point in Lat/Long coordinates in degrees and minutes to 4 decimal places
- Bonus Waypoints: Random placement in Lat/Long coordinates in degrees and minutes to 4 decimal places
  - These are using 18" orange plastic traffic cones.
- Manditory Waypoints: Random placement in Lat/Long coordinates in degrees and minutes to 4 decimal places
  - These are using 18" orange plastic traffic cones.
- Finish line: Unknown at this time, likely the same as a Waypoint.
- Datum specifically is WGS84.

### Travel Distance of the race
- Total straight-line distance between the start and destination is <= 1,000 ft.
- Total path through all waypoints avoiding obstacles could be well over 1,000 ft.

### This means the car will need to be able to:
- Needs to be able to detect orange cones
- Needs to be able to add lat/lon coordinates to the map being created.
- Needs to be able to mark certain obstacles as _avoid_ such as gullies that could trap it.
- Needs to have enough battery life to drive over 1,000 feet.
  - It would be idea to stress test this to understand the batteries limits.
  - If the best path goes beyond the battery stress test then it need to pick a smaller path that will be within the battery stress test.
  - NOTE: the mandetory waypoints will all have to fit within the stress test.
  - NOTE: Could look into a series of less important features to turn off in these bases for _battery saver_ mode.
- Take into account slip if using wheel encoders.
- (Optional) might need to take into account suspenssion if using wheel encoders

## Car requirements
- the robot must be fully autonomous (except for fail-safe device) during contest runs. Acceptable forms of this:
  - Wired tether operated by the handler walking alongside the robot
  - Some wireless contrivance operated by the handler
  - Some other mechanism, with prior permission from the event.
- the robot must weigh less than 50 pounds
- the robot must fit within a 4′ long x 4′ wide x 10′ envelope
- the robot’s center of mass must be lower than 4′ high
- the portion of the robot extending above 4′ high must fit within a 6″ long x 6″ wide x 6′ high column
  - this extension must not create a hazard (tipping or otherwise) to spectators or participants
  - the extension is aimed at giving sensors a better vantage point

### Hardware limitations
- Thus far there don't seem to be any real limitations here that I am concerned about. See [RC Car Design](Rc-Car-Design) for more details.


## Play Rules
- 15 minutes per attempt, 3 attempts, best score taken.
- Manditory waypoints AND possible order they may need to hit them in.
- The car may not run the course before the run.
- Contestants may take detailed measurements as they need to.
  - GPS
  - Cell-phone
  - tape measure
  - Even custom solutions for mapping, such as your own hardware, etc.
    - The way is this phrased it seems like we could setup a triangulation grid around the map. The pass that data to the RC car.
- Cars _may_ run the same course at the same time.
- Cars must touch the waypoint cones without moving them to get points.
- Cars can attempt different routes each attempt.
  - Gives the option of using an attempt for discovery then run the last two for best route.
    - Likely easiest solution would be to instead have a good mapping solution ahead of time, and load the map into the car so it only focuses on localization and dynamic object avoidence ( other cars ).

### This means the car will need to be able to:
- Finish 1,000+ ft drive under 15 min.
- Take a map as input that contains objects: 
  - manditory - order, size, location
  - bonus     - order, size, location
  - start     - order, size, location
  - obsticle. - order, size, location, weight ( how traversable it is ) 
- Object tracking - this is mainly for dynamic objects such as other cars.
  - Ideally just avoid them, slow down, drive around, etc.
- Must not move waypoints. meaning it has to slow down and bump them.

## Scoring
Cars will receive a score corresponding to the number of seconds needed to travel to the destination and any bonus waypoints. The robot with the lowest score on any individual run will win.

- Main Score - total seconds to get from start to destination
- Bonus Waypoints - each waypoint gets a modifier from 0.1 - 0.9 based on difficultly to get to. Example: 500 sec * 0.1 * 0.2 * 0.9 = 9 seconds.

While there is more detailed scoring on in regards to what it means to not touch cones, or to avoid them completely, the primary goal is the above.

### This means the car will need to be able to:
- Design a path based on the required order of manditory waypoints
- Design additional paths that could take in more or less risk based on bonus waypoints
- Calculate additional paths based on the other rules not mentioned above.
  - The aim here is for any of the following cases:
    - Nobody is completing the course
    - System determines the bettery will likely die before completing the min required course
    - Based on the scoring the best min score could still be acheived by skipping required waypoints
      - NOTE: Bonue waypoints are not added if manditory waypoints are skipped.


## Basic functionality needs
This is more of a checklist that I can come back to as needed. Please see [Software Design](Software-Design) or [Rc Car Design](Rc-Car-Design) for more details each peice of this.
- Diagnostics package
  - Battery life and estimated time left
  - CPU Tempurature
  - CPU and Memory usage
- Localization
  - GPS / Acceleromitor
  - Cameras
  - (optional) Wheel encoders
- Dynamic and static object tracking and avoidance
  - Cameras
    - Be able to see/track other cars
    - Be able to see/track waypoints
    - Be able to see/track known and unknown obstacles
- Kill swith
  - Starting with just a leash
- RC reciever for manually driving needs
