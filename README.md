This project is a working simulation of a vehicle radar sensor returning data from measurements on random points of static objects.
# To Run
Open 4 new terminal windows and use the source command to run ```source ~/sim_env/devel/setup.sh``` on each.
Then run each command on a different terminal window:
```
roslaunch vehicle_and_obj robot.launch
rosrun vehicle_and_obj node
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=broadcaster/cmd_vel
rosrun vehicle_and_obj listener
```
# Using the Simulation
Instructions for moving the vehicle model are given in the terminal where ```rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=broadcaster/cmd_vel``` was executed.
 <img width="404" alt="Screenshot 2024-08-05 at 12 43 56 PM" src="https://github.com/user-attachments/assets/329d973a-e089-45d4-84a3-99ce9fb39e65">

 Example of a working simulation:  
 <img width="527" alt="resultsImg2" src="https://github.com/user-attachments/assets/8e918306-eed1-43cb-8ba5-ae31adc16a21">  
 PointCloud data is represented as the white marks on the static object; the green marker is there for convenience to locate the most recently measured random point.
