# my_ft

## To build a map for a world
1) the world should be in the world folder
2) roslaunch my_ft my_map.launch world:=WORLD NAME
3) save the map after the world is fully explore

## modify map to extract waypoints
1) rosrun my_ft comvert_im.py THE_MAP_NAME_SAVED

## make full coverage of the world
1) roslaunch my_ft my_world.launch map_pgm:=THE_MAP_NAME_SAVED world:=WORLD NAME
