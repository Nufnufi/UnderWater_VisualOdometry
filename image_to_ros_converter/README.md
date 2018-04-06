# image_to_ros_converter
C++ code to display images on a rostopic at a chosen rate

A launch file is provided : image_to_ros_converter.launch
ros parameters:
    - start_image (int) : number of the first image to display
    - last_image (int) : number of the last image to display
    - path_to_image (string) : folder path to the images
    - image_extension (string) : image extension (jpg, png ...)
    - fps (double) : image publishing rate (fps = 1 for 1 image published per second)
    - topic_color (string) : name of the published topic for the color images
    - topic_grey (string) : name of the published topic for the greyscale images

