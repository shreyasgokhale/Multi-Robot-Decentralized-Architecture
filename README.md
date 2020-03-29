# Multi-Robot Decentralized Architecture

### This project is a part of Master Thesis: _"Decentralized Multi Robot Collaboration and Mapping"_ by [Shreyas Gokhale](https://shreyasgokhale.com/#).

## How to

Run

 ```docker-compose up --build --force-recreate```

This fetches all the base images, builds containers and starts 1 Simulator, 3 Robot, 1 Google Cartographer Server containers. 

You can interact with the containers from VNC on follwing adresses:

| Container | IP |
|---------|------------|
| Gazebo Simulator | `localhost:5925` |
| Cartographer Cloud | `localhost:5910` |
| Robot 0 | `localhost:5950` |
| Robot 1 | `localhost:5951` |
| Robot 2 | `localhost:5952` |


If you run the code on a VM / DigitalOcean Droplet, replace `localhost` with your `VM IP` and open corresponding ports for remote access.


Have a look at  `docker-compose.yaml`  file for more info.

You can start teleop node for turtlebot from any one of the robot contaiers to move around the robot

```roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch```


## Screenshots

### Gazebo Container
![Gazebo Container](/screenshots/Gazebo_Container.png)

### Robot Container
![Robot_Container](/screenshots/Robot_Container.png)

### Google Cartographer Cloud Container
![Google Cartographer Cloud Container](/screenshots/GC_Cloud_Container.png)


## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

## Acknowledgements
The basis for this architecture was [Ros-Simulation](https://github.com/microsoft/Ros-Simulation) project.

## License
[GPL-3](https://choosealicense.com/licenses/agpl-3.0/)