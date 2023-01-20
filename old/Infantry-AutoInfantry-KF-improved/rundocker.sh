#!/bin/sh
PWD=`pwd`
docker run -it --rm --privileged -v /dev:/dev -v $PWD:/home/code --name virtual_env shanzoom/opencv_env:1.0