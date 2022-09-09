#!/bin/bash

function parseCheck(){
    if [[ -z "$1" ]]; then
        echo "Missing necessary parameter, please reference ./run.sh --help"
        return 1
    fi
    return 0
}

RUNTIME=""
IMAGE=$(basename `git rev-parse --show-toplevel`)
DOCKER_VERSION=$(docker version --format '{{.Client.Version}}' | cut --delimiter=. --fields=1,2)


if [[ $DOCKER_VERSION < "19.03" ]] && ! type nvidia-docker; then
    RUNTIME="--gpus all"
else
    RUNTIME="--runtime=nvidia"
fi

while [[ $# -gt 0 ]]; do
    case "$1" in
        -v|--version)
            COMPONENT_VERSION_STRING="$2"
            shift
            shift
        ;;

        -s|--scenario)
            case "$2" in
                --from_local)
                    if parseCheck "$3" ; then
                        SCENARIO_PATH="$3"
                        SCENARIO=$(printf %s "$SCENARIO_PATH" | awk -F"/" '{print $NF}')
                        shift
                        shift
                        shift
                    else
                        exit 1
                    fi
                ;;
                --from_docker)
                    if parseCheck "$3" ; then
                        SCENARIO="$3"
                        shift
                        shift
                        shift
                    else
                        exit 1
                    fi
                ;;
            esac
        ;;

        -h|--help)
            echo "-v| --version [docker image tag]"
            echo "-s| --scenario --from_local  [path to scenario]"
            echo "-s| --scenario --from_docker [scenario name]"

            exit 0
    esac
done

if [[ -z "$COMPONENT_VERSION_STRING" ]]; then
    COMPONENT_VERSION_STRING=$("./get-component-version.sh")
fi

if [[ -z "$SCENARIO" ]]; then
    SCENARIO="HelloWorld"
fi

parseCheck "$SCENARIO_PATH"

echo "##### Running usdotfhwastol/$IMAGE:$COMPONENT_VERSION_STRING docker container with $SCENARIO scenario #####"

if [[ -z $SCENARIO_PATH ]]; then
    # run scenario which already exists in docker container 
    docker run \
    --rm -it\
    --gpus all\
    --net=host\
    -v /tmp/.X11-unix:/tmp/.X11-unix\
    -e DISPLAY=$DISPLAY\
    -e QT_X11_NO_MITSHM=1\
    --user=carma usdotfhwastol/$IMAGE:$COMPONENT_VERSION_STRING\
    ./mosaic.sh -s $SCENARIO
else
    # mount and run scenario from local to docker container
    docker run \
    --rm -it\
    --gpus all\
    --net=host\
    -v /tmp/.X11-unix:/tmp/.X11-unix\
    -v $SCENARIO_PATH:/opt/carma-simulation/scenarios/tmp_scenario \
    -e DISPLAY=$DISPLAY\
    -e QT_X11_NO_MITSHM=1\
    --user=carma usdotfhwastol/$IMAGE:$COMPONENT_VERSION_STRING\
    ./mosaic.sh -s tmp_scenario
fi
