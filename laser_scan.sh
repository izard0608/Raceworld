gnome-terminal -t "raceworld_1car_laser" -- bash -c 'source devel/setup.bash;roslaunch raceworld raceworld_1car_laser.launch;bash'
sleep 15
gnome-terminal -t "laser_scan" -- bash -c 'source devel/setup.bash;rosrun raceworld laser_scan.py;bash'
