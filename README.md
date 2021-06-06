# AutonomousDrone
This project is in active development by BITSRobocon.

### What Motivated us :
**[Amazon Prime Air](https://www.amazon.com/Amazon-Prime-Air/b?ie=UTF8&node=8037720011) -**
These days we are pretty habituated of home- delivery system through e-commerce platform, however there is a big dependency on delivery boys and vehicles for timely delivery of the items. We could potentially use Drones for last mile delivery of items. While current prevalent addressing mechanism such as lat/long and post code are good enough for humans, these won’t work for drone delivery as all houses in a multi-storey building will have same lat/long or post-code. Design a solution which can help drones to identify each address / flat as a separate unit and deliver the item accordingly.

Some of the exisiting autonomous UAV frameworks available is/are: [GAAS](https://gaas.gitbook.io/guide/software-realization-build-your-own-autonomous-drone/an-overview-of-gaas)


### **What we Aim for :**

1) Given a target location on the map, the drone must navigate to it autonomously avoiding the obstacles in between.
2) Extend the mapping to corridors and making the drone move from one chamber to another by manually giving the position of the chambers on the map.
3) Autonomous identification of the position of the chamber on the map and reaching its target.
(The use of april tags or other identification methods must be researched here).


The project is divided into several subsystems. [Have a look](https://github.com/BitsRobocon/AutonomousDrone/projects)

This is a fairly new project for us and we have only little experience in ROS and SLAM. Therefore, we are trying to document every choice we make in the form of [issues](https://github.com/BitsRobocon/AutonomousDrone/issues).
This will not only help us to take a look back at all our decisions and refer to them as and when needed but will hopefully also develop as a comprehensive guide for future enthusiasts. 
