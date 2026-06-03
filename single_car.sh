gnome-terminal -t "raceworld_1car" -- bash -c 'source devel/setup.bash;roslaunch raceworld raceworld_1car.launch;bash'
sleep 15
gnome-terminal -x bash -c 'source devel/setup.bash;rosrun raceworld traffic_light.py;bash'
sleep 3
gnome-terminal -t "single_car" -- bash -c 'source devel/setup.bash;rosrun raceworld single_car.py;bash'
