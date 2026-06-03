gnome-terminal -t "raceworld_1car" -- bash -c 'source devel/setup.bash;roslaunch raceworld raceworld_1car.launch;bash'
sleep 2
gnome-terminal -t "key_op" -- bash -c 'source devel/setup.bash;rosrun raceworld key_op.py;bash'
