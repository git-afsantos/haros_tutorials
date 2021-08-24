# Docker Image for the HAROS Tutorial

This directory contains files to build and run a Docker image with a functional installation of HAROS.
It is intended to be used with the source code found in this repository and the target application for analysis.

Once you have Docker available in your system, navigate to this directory.

```bash
cd haros_tutorials/docker
```

Then, either execute the following command or run the [`build.sh` script](https://github.com/git-afsantos/haros_tutorials/blob/master/docker/build.sh), which does the same thing.

```bash
docker build -t fictibot .
```

This command will produce a Docker image called `fictibot`.
The files [`configs.yaml`](https://github.com/git-afsantos/haros_tutorials/blob/master/docker/configs.yaml) and [`catkin.sh`](https://github.com/git-afsantos/haros_tutorials/blob/master/docker/catkin.sh) found in this directory are needed during the build step, to be copied to the image's file system.

To run the image, use the [`run.sh` script](https://github.com/git-afsantos/haros_tutorials/blob/master/docker/run.sh) or enter the following command.

```bash
docker run -it \
    --tmpfs /tmp \
    -p 8080:8080 \
    --mount type=bind,source=..,target=/root/ws/src/haros_tutorials,readonly \
    fictibot
```

This command requires that the TCP port `8080` is available in your system, for HAROS to use as an HTTP server (for visualization).
If you would rather use another port, e.g., 6480, change the `-p 8080:8080` line to `-p 8080:6480`.

> **Note:** the Docker container does not automatically open your web browser for visualization.
> Whenever HAROS runs the visualization server, you have to manually open your web browser at `localhost:8080` (or your selected port).

In addition, the command binds the `/tmp` directory of the container to a `tmpfs` filesystem to improve efficiency (HAROS generates temporary files during its analysis) and it binds the `/root/ws/src/haros_tutorials` directory of the container to the root of this repository in the host system, in read-only mode.
In essence, this means that you need only one copy of this repository and its source code, which shall remain in the host machine.
The Docker container running HAROS has read access to the repository, and will treat it as if it were a native directory within the container, so that HAROS has access to the source code for analysis.

Once the container is running, it should start in the `/root/ws` directory, which is the root of the catkin workspace.
In this directory, run the `catkin.sh` script to build Fictibot's source code and `source` the workspace.

```bash
./catkin.sh
source devel/setup.bash
```

With this, the container is ready to run `haros`.
Try, for instance, the [`fictibot.sh` script](https://github.com/git-afsantos/haros_tutorials/blob/master/scripts/fictibot.sh):

```bash
cd src/haros_tutorials
./scripts/fictibot.sh
```
