docker run -it \
    --env="DISPLAY=host.docker.internal:0" \
    --name rssa \
    --net=host \
    --privileged \
    --mount type=bind,source=$HOME/rssa_shared,target=/home/rssa \
    rssa \
    bash
    
docker rm rssa
