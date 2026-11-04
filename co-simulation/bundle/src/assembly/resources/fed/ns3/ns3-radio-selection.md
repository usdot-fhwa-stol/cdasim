# Selecting the ns-3 radio technology

CDASim can run the ns-3 federate with either DSRC/IEEE 802.11p or 5G NR V2X sidelink support. The selected Docker image and the scenario's ns-3 configuration files must use the same radio technology.

| Radio technology | ns3-federate image | Scenario configuration |
| --- | --- | --- |
| DSRC (default) | `usdotfhwastol/ns3-federate:latest-dsrc` | Standard scenario, for example `Tiergarten` or `Town04_carma_messenger` |
| 5G NR | `usdotfhwastol/ns3-federate:latest-5g-nr` | Matching `_NR` scenario, for example `Tiergarten_NR` or `Town04_carma_messenger_NR` |

## Build the ns3-federate image

From the `ns3-federate` repository, use its common image build script:

```bash
# DSRC is the default when no radio technology is supplied.
./docker/build-image.sh
./docker/build-image.sh --radio-tech dsrc

# Build 5G NR.
./docker/build-image.sh --radio-tech 5g-nr
```

These commands create the `latest-dsrc` or `latest-5g-nr` technology-qualified tag in addition to the component-version tag.

## Select the image in CDASim

Set the ns-3 federate's `dockerImage` in [`runtime.json`](../co-simulation/bundle/src/assembly/resources/etc/runtime.json). Use the DSRC image:

```json
"dockerImage": "usdotfhwastol/ns3-federate:latest-dsrc"
```

or the 5G NR image:

```json
"dockerImage": "usdotfhwastol/ns3-federate:latest-5g-nr"
```

Rebuild the CDASim bundle/image after changing the source runtime configuration. For a temporary test inside an existing CDASim container, make the equivalent change in `/opt/carma-simulation/etc/runtime.json`.

The `port` value in `runtime.json` does not need to change. When CDASim starts ns3-federate as a Docker federate, the image uses ports `40001` and `40002` for its two MOSAIC communication channels.

## Use matching scenario configuration files

Each scenario supplies both files below in its `ns3` directory:

```text
ns3/ns3_config.json
ns3/ns3_federate_config.xml
```

Always switch or copy these two files together. Standard scenarios contain DSRC/WAVE attributes, while `_NR` scenarios contain NR sidelink attributes. For example:

```bash
# DSRC
./mosaic.sh -s Tiergarten

# 5G NR
./mosaic.sh -s Tiergarten_NR
```

Do not run the DSRC image with an `_NR` `ns3_federate_config.xml`. NR-only attributes such as `ns3::MosaicNodeManager::numExtraRadioNodes` are not available in the DSRC federate and will abort it during startup.

## Running CDASim inside Docker

When a CDASim container uses the host Docker socket to start an ns3-federate child container, let the child share the CDASim container's network namespace. The container name and `MOSAIC_DOCKER_NETWORK` value must match:

```yaml
services:
  carma-simulation:
    container_name: carma-simulation
    environment:
      - MOSAIC_DOCKER_NETWORK=container:carma-simulation
    volumes:
      - /var/run/docker.sock:/var/run/docker.sock
      - /opt/carma-simulation/tmp:/opt/carma-simulation/tmp
      - /opt/carma-simulation/logs:/opt/carma-simulation/logs
```

`DockerClient` passes this value to the ns3-federate launch as `--network container:carma-simulation` and omits `-P`. Both containers then use `localhost:40001` and `localhost:40002` for the ns-3 communication channels.

The same-path `/opt/carma-simulation/tmp` bind mount is also required. Docker resolves child-container bind-mount sources on the host, so this mount ensures that ns3-federate receives the configuration files that CDASim just deployed rather than stale host files.

Check `MOSAIC.log` when troubleshooting. A container-network launch should resemble:

```text
Resolved docker network: container:carma-simulation
Executing docker run ... --network container:carma-simulation --name ns3-federate ...
```

If the log contains `-P` and CDASim itself is running in a bridge container, the ambassador may attempt to connect to a host-published port through the CDASim container's own `localhost`, resulting in `Connection refused`.
