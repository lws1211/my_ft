#!/usr/bin/env python3
import sys
import cv2 as cv
import rospy, rospkg
src = None
def main(image, name, path):
    global src
    src = cv.imread(image)
    
    erosion(name, path)
def erosion(name, path):
    # erosion_size = cv.getTrackbarPos(title_trackbar_kernel_size, title_erosion_window)
    # erosion_shape = morph_shape(cv.getTrackbarPos(title_trackbar_element_shape, title_erosion_window))
    # print(f"{path}/map/{name}")
    ero = 2
    element = cv.getStructuringElement(0, (2 * 2 + ero, 2 * ero + 1),
                                       (ero, ero))
    
    print(f"{path}/map/{name}")
    erosion_dst = cv.erode(src, element)
    cv.imwrite(f"{path}/map/{name}", erosion_dst)
if __name__ == "__main__":
    # pgm_name = rospy.get_param("/map_pgm")
    pgm_name = None
    args = rospy.myargv(argv=sys.argv)
    if len(args) != 2:
        print("error, expected name of the file. example: rosrun my_ft convert_im.py mymap")
    pgm_name = args[1]
    pgm_name_ = pgm_name +".pgm"
    yaml_name = f"{pgm_name}.yaml"
    
    rospack = rospkg.RosPack()
    path = rospack.get_path("my_ft")
    new_name = str(pgm_name) + "2" +".pgm"
    print(pgm_name, new_name)
    new_name_y = str(pgm_name) + "2" +".yaml"
    main(f"{path}/map/{pgm_name_}", new_name, path)
    with open(f"{path}/map/{yaml_name}", "r") as file:
        lines = file.readlines()
    with open(f"{path}/map/{new_name_y}", "w") as file:
        lines[0] = "image: " + new_name + "\n"
        file.writelines(lines)
