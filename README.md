# DP3_Chart_Simulation

The A&E demonstration is to prioritise incoming casualties from disaster sites. The A&E ward consists of multiple robots such as MiR, Sesto, MissyBot, RoMiO Bot and the TrolleyBed(with Bolt-On Kit).

In typical A&E scenario, the robots will perform their task as per normal according to the task request. However, when the emergency button is activated, all the robots are to pause their assigned task and navigate to the nearest parking spot.

This simulation also shows double deconfliction between two robots and the incoming trolley bed from the ambulance. The first priority will always go to the trolley bed.

# Gazebo Simulation
The robots are using the Full Control Fleet Adapter to navigate and accept loop requests while the trolley bed use Read-Only Fleet Adapter to reenact manual pushing.

# Fleet Adapters for individual robots & trolleybed
Full Control Fleet Adapter:
- MiR Robot
- MissyBot
- RoMiO Bot

Traffic Light Control Fleet API:
- Sesto Magnus Robot

Manual Pushing:
- Trolley Bed(with Bolt-On Kit)

# Reference

Refer to this link for System Requirement, Installation & Tutorials
- https://github.com/sharp-rmf/rmf_demos 
- https://osrf.github.io/ros2multirobotbook/integration_fleets.html

# Starting and Alternate Parking Points

![](https://github.com/sharp-rmf/DP3_Chart_Simulation/blob/assets/Initial%20Point%20%26%20Parking%20Spots.png)

# Emergency Route

![](https://github.com/sharp-rmf/DP3_Chart_Simulation/blob/assets/Emergency.png)
