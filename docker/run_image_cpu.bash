#IMAGE=jumping_from_pixels:latest
#IMAGE=gmargo11/jumping_from_pixels:release
IMAGE=gmargo11/jumping_from_pixels:public
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]
    then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

docker run -it \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --volume="/data/pulkitag/models/gabe/data2:/data" \
    --volume="$PWD/../:/workspace/jumping-from-pixels" \
    --privileged \
    --net=host \
    ${IMAGE}

