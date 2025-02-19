
xhost +local:docker
docker run -it \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix" \
    --name rssa \
    --net=host \
    --privileged \
    --mount type=bind,source=$HOME/rssa_shared,target=/home/rssa \
    rssa \
    bash
    
docker rm rssa
