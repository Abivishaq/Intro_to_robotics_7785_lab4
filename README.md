# Intro_to_robotics_7785_lab4
# Setup on turtlebot
git clone the repo to your local system
```
git clone <url>
```
### connect to the turtlebot in the "files" the ubuntu file explore
1. open files
2. click on "other Locations" on the left panel
3. At bottom you should see Connect to Server
4. In Enter server address: ssh://burger@<ip-address>
5. copy paste the folder in src directory of our workspace (can't remember the name)
6. in a terminal ssh in to the tutle bot
7. navigate to the workspace directort :
   ```
   cd <workspace_name>_ws
   ```
### running 
For each node you need to ssh into the turtle bot and source the workspace
ssh:
```
ssh burger@<ip-adress>
```
sourceing workspace:
```
cd <workspace_name>_ws
source install/setup.bash
```
Run the following node on seperate terminals
```
ros2 run team7_lab4 chase_goal 
```
```
ros2 run team7_lab4 goToPos 
```
```
ros2 run team7_lab4 corner_pointer
```
```
ros2 run team7_lab4 path_planner
```
