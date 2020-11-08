## README

Build the image by running `docker build . -t sfusion` in this directory.

Run the container with

`docker run -v ~/git/slambench3:/sb3 --device /dev/nvidia0:/dev/nvidia0
--device /dev/nvidiactl:/dev/nvidiactl -v
<path_to_host_dataset_directory>/datasets:/datasets  --net=host --gpus=all
--env="DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix:rw -it sfusion`

The image should build the slambench deps and semanticfusion.
Does not build any datasets.

In order to run semanticfusion, use:

` cd /slambench3 && ./build/bin/pangolin_loader -i /datasets/bathroom_0005.slam -load
./build/lib/libsemanticfusion-original-library.so -d
benchmarks/semanticfusion/src/original/semantic_dictionary.txt`


Will probably need to run

`xhost +local:root` to allow access to the X server from the docker.


### Check if everything works

There might be problems with Nvidia/CUDA forwarding. Check if it works in
the container using

` cd /opt/cuda/samples/1_Utilities/deviceQuery/ && make && ./deviceQuery`

Should list the CUDA device and end with the line `Result = PASS`.

Check X forwarding by running `glxgears`
