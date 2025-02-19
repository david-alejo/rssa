docker run -it ^
--env="DISPLAY=host.docker.internal:0" ^
--name rssa ^
--net=host ^
--privileged ^
--mount type=bind,source=C:\Users\username\rssa_shared,target=/home/rssa ^
rssa ^
bash
    
docker rm rssa
