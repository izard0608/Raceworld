gnome-terminal -x bash -c 'roscore'
sleep 2
gnome-terminal -x bash -c 'source devel/setup.bash;rosrun raceworld pub1.py;bash'
gnome-terminal -x bash -c 'source devel/setup.bash;rosrun raceworld sub1.py;bash'
