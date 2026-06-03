gnome-terminal -x bash -c 'roscore'
sleep 2
gnome-terminal -x bash -c 'source devel/setup.bash;rosrun raceworld pub2;bash'
gnome-terminal -x bash -c 'source devel/setup.bash;rosrun raceworld sub2;bash'
