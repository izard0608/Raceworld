gnome-terminal -t "raceworld_1car" -- bash -c 'source devel/setup.bash;roslaunch raceworld raceworld_1car.launch;bash'
sleep 15
gnome-terminal -t "follow" -- bash -c 'source devel/setup.bash;rosrun raceworld follow.py;bash'
