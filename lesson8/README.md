[< Previous lesson](../lesson7/) -- [**Main Readme**](../README.md)

# Lesson 8 - Testing in the CARLA simulator

In this final lesson, you will run the whole framework from the previous lessons in closed loop inside the CARLA simulator: the simulated world reacts to your vehicle, and your vehicle must react to the world.

Two tools are used for the closed-loop validation:
* [**CARLA**](https://carla.org/) - an open-source autonomous driving simulator. It renders the world via provided map files (and we will use our own Tartu map), simulates the physics and the sensors (lidar, cameras), and feeds them to your nodes through ROS topics.
* **Visual Scenario Editor (VSE)** - a graphical tool for creating and re-playing driving scenarios in CARLA: NPC vehicles and pedestrians with routes and triggers, traffic light sequences and weather. See the [VSE repository](https://github.com/UT-ADL/visual-scenario-editor) and [how to use the editor](https://github.com/UT-ADL/visual-scenario-editor/blob/main/tutorial.md).

You will first verify that your framework can drive in CARLA, then run it through a prepared VSE scenario, and finally design scenarios yourself where your framework fails.

### Expected outcome
* Understanding how the full autonomous driving stack behaves in a closed-loop simulation
* Exploring the limits of the framework you built


## 1. Run your stack in CARLA

The launch file [lesson8.launch](launch/lesson8.launch) connects your nodes from the previous lessons to CARLA. There is no bag playback: the localization comes from the simulator, and the vehicle commands from your `pure_pursuit_follower` steer the car in the simulation.

By default the detected objects and traffic light statuses come from the simulator's ground truth instead of your perception nodes - simulating the lidar and the cameras is very heavy, and running the perception pipeline on them can slow the simulation down to a crawl. Your planner and controller are still the ones driving. If your machine can afford it, you can enable your own perception with `detector:=cluster` (lesson 5 nodes on the simulated lidar) and/or `tfl_detector:=yolo` (lesson 7 nodes on the simulated cameras).

##### Instructions
1. Start the CARLA simulator:
    ```
    $CARLA_ROOT/CarlaUE4.sh -prefernvidia -RenderOffScreen
    ```
2. In another terminal, launch your stack:
    ```
    roslaunch autoware_mini_tutorial lesson8.launch
    ```

##### Validation
* RViz opens with the Tartu map and the ego vehicle placed in the simulated city
* The `Carla image view` panel shows the third-person view of the ego vehicle in the simulated world
* Place a goal on the map - the vehicle drives to it


## 2. Run the demo scenario

A driving scenario adds actors to the otherwise empty world: NPC vehicles and pedestrians that spawn, move and react when triggered, and traffic lights that switch according to the scenario triggers. You will run the prepared demo lap scenario and see whether your framework survives traffic.

When your stack is running, VSE automatically detects your ego vehicle and hands the driving over to it - the scenario provides the destination, the other actors and the evaluation.

##### Instructions
1. With `lesson8.launch` running, start VSE and open the `tartu_demo` map. When VSE first launches, it will ask to select the agent's behavior logic. Navigate to `autoware_mini/nodes/platform/carla/` and select `carla_minimal_agent.py`.
2. Open the scenario (`Scenario` menu -> `Open`): `shared/data/scenarios/tartu_demo_route_simplified.json` from the tutorial folder
3. Press **Play**. Note: if your machine has less than 10 Gb VRAM slowdowns are expected.

##### Validation
* The goal appears in RViz automatically and the vehicle starts driving the demo lap
* NPC vehicles and pedestrians act out the scenario around the ego vehicle
* When the run ends, VSE shows a results window scoring the drive (collisions, red light violations, route completion); the same results are also saved as a text file next to the scenario JSON


## 3. Create three failure cases

Your framework from the previous lessons is a simplified one. Remember all limitations that were discussed through the lessons. In this final task you will demonstrate these limits: create three scenarios where your framework fails.

##### Instructions
1. Copy `tartu_demo_route_simplified.json` (e.g. to `failure_case_1.json`) and modify it in VSE - move, add, retime or reroute actors and triggers until your stack demonstrably fails, while a careful human driver would still manage
2. For every failure case, think of a specific change to the framework that would fix it. You do not need implement the fix. The three cases should have three different proposed fixes.
3. Create a `lesson8/scenarios/` folder in your repository and commit the three scenario JSONs there
4. Fill in the three descriptions below: what happens in the scenario, how your framework fails, and what change to the framework would fix it. Add screenshots if needed.
5. Commit and push everything, and be ready to demonstrate your failure cases at the practice session

##### Failure case 1
**Scenario:** `failure_case_1.json` — a `vehicle.mitsubishi.fusorosa` truck is parked (no route, so it stays at spawn) in the ego's lane ~100 m ahead, with a personal trigger ~40 m before it so it spawns before the ego arrives.

**How it fails:** the ego brakes and comes to a full stop behind the truck, then waits there for the rest of the run. The local planner is longitudinal-only (speed), the pure pursuit follower stays on the fixed global path, and the route planner never plans lane changes, so the ego cannot go around. The run ends in a timeout / route-not-completed failure.

**Proposed fix:** a lateral avoidance / lane-change behaviour planner that detects a stationary obstacle blocking the lane and re-routes around it, instead of only reducing speed.

##### Failure case 2
**Scenario:** `failure_case_2.json` — an ambulance (`vehicle.ford.ambulance`) is routed to merge into the ego's lane from the oncoming lane / a side road just a few metres ahead, triggered so the cut-in happens while the ego is already inside its braking distance.

**How it fails:** the ego detects the cutting-in ambulance too late. With `default_deceleration` 1.0 m/s² and `braking_reaction_time` 1.6 s, the required stopping distance exceeds the remaining gap, so a collision is unavoidable.

**Proposed fix:** motion prediction of other actors combined with emergency braking — earlier detection/anticipation of converging trajectories and a higher (emergency) deceleration when a collision is imminent.

##### Failure case 3
**Scenario:** `failure_case_3.json` — an existing traffic light on the ego's route is re-timed in VSE (large trigger radius, short green ~2–3 s, then yellow 1 s, red). The sequence starts when the ego enters the trigger circle, so the light turns red right as the ego commits at the stop line.

**How it fails:** the stack only reacts to the light's instantaneous status; it has no notion of the upcoming phase change. When the light turns red the ego is already too close to the stop line to brake in time and crosses on red — a red-light violation. A careful human driver sees the yellow and brakes early.

**Proposed fix:** traffic-light phase prediction / time-to-red logic — anticipate green→yellow→red transitions and start decelerating on yellow or on a looming red, not only when the red is already active.
