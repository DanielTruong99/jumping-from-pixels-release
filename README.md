## Setup Instructions

This repository contains the code for Learning to Jump from Pixels (https://openreview.net/forum?id=R4E8wTUtxdl) and a docker image definition for easy setup of dependencies.

*Build the Docker Image*

```
git clone git@github.com:Improbable-AI/jumping-from-pixels.git
cd jumping-from-pixels/docker && python3 docker_build.py
```


Once you have the docker image, you can start and enter a container by running the script at `docker/run_image_cpu.bash`.

(If you have `nvidia-docker` set up, you can run `docker/run_image.bash` to access GPU from within the container.)

## Testing the MPC Controller

To test out the MPC controller, first enter the docker container as described above. Then, run:

```
python3 examples/controller_demo.py
```

If everything works correctly, you will see a render of the mini cheetah trotting at constant speed in pybullet. `controller_demo.py` includes some useful examples of passing data between the controller and simulator!

## Evaluating the Jumping from Pixels policy

A few pretrained models are included with this repository for trotting, pronking, and variable pronking. They are: %TODO ADD PATHS HERE

You will find an evaluation script at `examples/evaluate_dic_trot.py`. Run it with:

```
python3 examples/evaluate_DIC_trot.py
```


## Training the Jumping from Pixels policy

To train a trotting policy with heightmaps, run:

```
python3 examples/train_DIC_trot.py
```
