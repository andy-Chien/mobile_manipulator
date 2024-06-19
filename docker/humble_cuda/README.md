## 1. Build image using docker file:
    ```
    # cd to this directory first
    docker build --build-arg="USER_NAME=<USER_NAME>" --build-arg="USER_PASSWD=<USER_PASSWD>" --build-arg="WS_NAME=<WS_NAME>" -t <IMAGE_NAME:TAG> <PATH>
    ```
    for example:
    ```
    # cd to this directory first
    docker build --build-arg="USER_NAME=ubuntu" --build-arg="USER_PASSWD=ubuntu_password" --build-arg="WS_NAME=colcon_ws" -t delta/ubuntu-cuda-ros:22.04-11.8-humble .
    ```
## 2. Run the image
    ```
    docker run -it --privileged -p <PORT> --security-opt seccomp=unconfined --shm-size=<MEM_SIZE> --gpus all -v /dev/:/dev/:rw -v <ABS_PATH_TO_LOCAL_WS>/src/:<ABS_PATH_TO_CONTAINER_WS>/src/:rw -v /tmp/.X11-unix/:/tmp/.X11-unix:rw --env="DISPLAY" --name <CONTAINER_NAME> <IMAGE_NAME:TAG> bash
    ```
    example:
    ```
    docker run -it -d --privileged -p 6080:80 --security-opt seccomp=unconfined --shm-size=512m --gpus all -v /dev/:/dev/:rw -v /home/<LOCAL_USER_NAME>/colcon_ws/src/:/home/ubuntu/colcon_ws/src/:rw -v /tmp/.X11-unix/:/tmp/.X11-unix:rw --env="DISPLAY" --name ubuntu_cuda_ros delta/ubuntu-cuda-ros:22.04-11.8-humble
    ```

## 3. Restart the container
    ```
    docker stop <CONTAINER_NAME>
    docker start <CONTAINER_NAME>
    docker exec -it <CONTAINER_NAME> bash
    ```