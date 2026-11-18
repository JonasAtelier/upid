# UPID

UPID is a portable C99 PID controller for embedded feedback loops. Give it a target, a sensor reading, and the elapsed time. It returns an output within the limits you set. The library fits in one header and one source file, uses no heap allocation, and leaves the clock and hardware I/O to your application.

**Contents** · [Run the heater example](#run-the-heater-example) · [Use it in your project](#use-it-in-your-project) · [Choose a tuning parameter](#choose-a-tuning-parameter) · [Reference](#reference)

![The feedback loop from target to controller, actuator, and measurement](assets/control-loop.svg)

## Run the heater example

With Git and a C99 compiler installed, build the simulation on your computer.

```sh
git clone https://github.com/JonasAtelier/upid.git
cd upid
cc -std=c99 -Wall -Wextra -Iinclude examples/basic_pid.c src/upid.c -lm -o basic_pid
./basic_pid
```

The simulated heater starts at 20 and aims for 50. Each line shows the measurement and controller output. With the example gains, the final line reads:

```text
settled at 50.016 (setpoint 50.0)
```

Try changing `cfg.kp` from `2.0f` to `3.0f` and rebuilding. On the first sample, the output rises from 60.150 to 90.150. Both runs approach the same target, but the stronger proportional gain pushes harder at the start. [Heater example](examples/basic_pid.c) · [Measured tuning results](TUNING.md)

## Use it in your project

Compile [src/upid.c](src/upid.c) with your application and add [include/](include/) to its include path. Create one controller per loop.

| When | Call | What to supply |
| --- | --- | --- |
| At startup | `upid_cfg_init(&cfg)` | Set the gains, output range, and direction after loading the defaults. |
| Once per controller | `upid_create(&pid, &cfg)` | Check for `UPID_OK` before starting the loop. |
| Each sample | `upid_spin(&pid, target, measurement, dt_s)` | Use elapsed time in seconds, e.g. `0.01f` for 10 ms. |
| After a successful update | `upid_get_output(&pid)` | Apply the result to your actuator. |

## Choose a tuning parameter

| What you observe | Where to start |
| --- | --- |
| The output responds weakly to a large error | Adjust `kp`, the response to the current error. |
| The measurement settles short of its target | Adjust `ki`, which accumulates persistent error. |
| An inertial system overshoots or oscillates | Explore `kd` and its measurement-noise filter, `d_filter_tau`. |
| The actuator saturates or changes too abruptly | Set `o_min`, `o_max`, and the output slew limit `o_rate_max`. |

The [tuning guide](TUNING.md) includes gain sweeps, measured overshoot, and a troubleshooting table.

## Reference

[API and return codes](include/upid.h) · [Releases](https://github.com/JonasAtelier/upid/releases) · [MIT license](LICENSE) · [Jonas Atelier library index](https://github.com/JonasAtelier/jonas-atelier-libraries#library-search)
