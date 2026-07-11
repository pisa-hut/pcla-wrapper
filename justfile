# Default host directory for the selected PCLA agent weights. Override it on
# recipes that accept a weights parameter.
selected_weights := env_var_or_default("PISA_DATA_DIR", "/PISA_DATA_DIR") + "/weights/plant_pretrained"

# Build the common runtime without model weights.
build-common-slim:
    docker build --target common-slim -t pcla-wrapper:common-slim .

# Validate common-slim dependencies and the selected checkpoint.
validate-common-slim weights=selected_weights:
    docker run --rm --gpus all \
        -v "{{weights}}:/mnt/weights:ro" \
        pcla-wrapper:common-slim \
        /app/scripts/validate_common_runtime.py --check-weights

# Load one agent and checkpoint without starting CARLA.
smoke-common-agent agent="carl_plant_3" weights=selected_weights:
    docker run --rm --gpus all \
        -v "{{weights}}:/mnt/weights:ro" \
        pcla-wrapper:common-slim \
        /app/scripts/smoke_common_agent.py "{{agent}}"

# Build and run common-slim with the local TYMS map directory.
run_t: build-common-slim
    docker run --gpus all --rm \
    --network host -it \
    -v /opt/sbsvf/map/tyms/xodr:/mnt/map/xodr \
    -v {{selected_weights}}:/mnt/weights:ro \
    -v /tmp/.X11-unix:/tmp/.X11-unix -v ~/.Xauthority:/root/.Xauthority \
    -e DISPLAY pcla-wrapper:common-slim

# Build and run common-slim with the local Frankenburg map directory.
run_f: build-common-slim
    docker run --gpus all --rm \
    --network host -it \
    -v /opt/sbsvf/map/frankenburg/xodr:/mnt/map/xodr \
    -v {{selected_weights}}:/mnt/weights:ro \
    -v /tmp/.X11-unix:/tmp/.X11-unix -v ~/.Xauthority:/root/.Xauthority \
    -e DISPLAY pcla-wrapper:common-slim
