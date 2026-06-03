gnome-terminal -t "raceworld_1car" -- bash -c 'source devel/setup.bash;roslaunch raceworld raceworld_1car.launch;bash'
sleep 15
gnome-terminal -t "stop" -- bash -c 'source devel/setup.bash;rosrun raceworld stop.py;bash'
