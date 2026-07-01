# Lifecycle

## Init

Init:

1. Finalizes an unfinished previous scenario without stopping a reusable CARLA
   process.
2. Stores output directory, config, and `dt`.
3. Validates config and the PCLA registry name.
4. Starts CARLA when configured.
5. Retries connection within one total timeout.
6. Clears stale dynamic actors from a reused world.

Large PCLA models are not imported or loaded during constructor or Init.

## Reset

Reset:

1. Finalizes the previous PCLA instance and wrapper actors.
2. Resolves the case output and creates its writable PCLA runtime directory.
3. Validates map name, initial ego observation, and goal.
4. Loads `<xodr_root>/<map_name>.xodr`.
5. Reuses the generated world when the map/path match and reuse is enabled.
6. Spawns the wrapper-owned ego.
7. Applies synchronous/no-rendering settings and `InitRequest.dt`.
8. Configures `CarlaDataProvider`.
9. Validates or atomically generates route XML.
10. Loads the PCLA model and its sensors from the writable runtime directory.
11. Enables rendering when the agent attached camera sensors.
12. Runs configured sensor warm-up ticks.
13. Runs the initial observation through the normal Step path.

Any partial failure triggers cleanup before the exception is returned.

## Step

Step reads ego from `observation.ego` and non-ego actors from
`observation.agents`. Agent list order is presentation-only. It:

1. Synchronizes ego and non-ego shadow actors.
2. Ticks or waits for CARLA exactly once.
3. Gets one snapshot.
4. Updates `CarlaDataProvider`.
5. Passes that same snapshot to PCLA/GameTime.
6. Returns `THROTTLE_STEER_BREAK`.

Reset observation time is `0 ns`. Later Step timestamps must equal the absolute
`step_index * Init.dt` grid. The wrapper gives PCLA an episode-relative snapshot
timestamp, so reuse of a CARLA process does not leak elapsed time across resets.

PCLA construction, action calls, and cleanup run with
`<ResetRequest.output_dir>/pcla_runtime` as their temporary current working
directory by default. The original process directory is restored after every
call.

PCLA exceptions set the fatal quit message and are not converted to normal zero
control. A `None` action follows `action_none_timeout_seconds`.

## Ownership

| Resource | Owner |
| --- | --- |
| CARLA process | Wrapper only when it launched the process |
| Ego actor | Wrapper |
| Observation-controlled non-ego actors | Wrapper |
| PCLA real and pseudo sensors | Current PCLA instance |
| Generated route file | Current reset output directory |
| Relative PCLA agent outputs | Current PCLA runtime directory |

Cleanup forces the world asynchronous and clears fixed delta time before actor
destruction. It does not delete traffic lights, static props, unrelated
sensors, or externally owned CARLA processes.

## ShouldQuit And Stop

ShouldQuit reports:

- route endpoint reached,
- owned CARLA process exit and return code,
- fatal PCLA error,
- finalized/stopped state.

Stop is idempotent. It cleans the PCLA agent/sensors and wrapper actors, then
releases client/world/model references. A wrapper-owned CARLA process remains
running for the next Init/Reset cycle and exits with the container. This keeps
one CARLA server per container instead of restarting it between scenarios.
