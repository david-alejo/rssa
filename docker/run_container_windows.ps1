docker run -it `
--env="DISPLAY=host.docker.internal:0" `
--name rssa `
--net=host `
--privileged `
--volume "/tmp/.X11-unix/:/tmp/.X11-unix/" `
--volume="$HOME/.Xauthority:/root/.Xauthority:rw" `
--mount type=bind,source=$HOME\rssa_shared,target=/home/rssa `
rssa `
bash
    
docker rm rssa
