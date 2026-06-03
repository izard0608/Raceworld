gnome-terminal -t "raceworld_4cars" -- bash -c 'source devel/setup.bash;roslaunch raceworld raceworld_4cars.launch;bash'
sleep 15
gnome-terminal -t "lane" -- bash -c 'source devel/setup.bash;rosrun raceworld lane.py;bash'
gnome-terminal -t "platoon1" -- bash -c 'source devel/setup.bash;rosrun raceworld platoon.py "car1" "car2" "base_link2" "follow1";bash'
gnome-terminal -t "platoon2" -- bash -c 'source devel/setup.bash;rosrun raceworld platoon.py "car2" "car3" "base_link3" "follow2";bash'
gnome-terminal -t "platoon3" -- bash -c 'source devel/setup.bash;rosrun raceworld platoon.py "car3" "car4" "base_link4" "follow3";bash'
