gnome-terminal -t "raceworld_2cars" -- bash -c 'source devel/setup.bash;roslaunch raceworld raceworld_2cars.launch;bash'
sleep 15
gnome-terminal -t "lane" -- bash -c 'source devel/setup.bash;rosrun raceworld lane.py;bash'
gnome-terminal -t "platoon" -- bash -c 'source devel/setup.bash;rosrun raceworld platoon.py "car1" "car2" "base_link2" "follow1";bash'
