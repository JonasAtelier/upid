# Universal PID Controller Library Design

Date: 2026-08-05

## Purpose

UPID will be a small, production-ready PID controller library for:

- Arduino on ESP32, AVR, SAMD, and RP2040
- Native ESP-IDF
- Host computers used for deterministic tests

The controller core will be portable C++11. It will not depend on Arduino,
ESP-IDF, a platform clock, dynamic allocation, exceptions, or RTTI.

## Version 1 Scope

Version 1 includes:

- Proportional, integral, and derivative control
- Explicit elapsed time supplied by the caller
- Output limiting
- Conditional-integration anti-windup
- Derivative-on-measurement
- First-order derivative filtering
- Manual and automatic modes
- Bumpless transfer from manual to automatic
- Direct and reverse controller direction
- Runtime validation with status codes

Feed-forward, setpoint weighting, cascade control, and autotuning are outside
the version 1 scope.

## Architecture

One implementation will serve all supported platforms:

```text
                    +----------------------+
                    | User application     |
                    +----------+-----------+
                               |
                    setpoint, measurement, dt
                               |
                    +----------v-----------+
                    | upid::Controller      |
                    | Portable C++11 core   |
                    +----------+-----------+
                               |
                         output + status
                               |
           +-------------------+-------------------+
           |                   |                   |
    +------v------+      +-----v------+     +------v------+
    | Arduino     |      | ESP-IDF    |     | Host tests |
    | packaging   |      | component  |     | and tools  |
    +-------------+      +------------+     +-------------+
```

Planned repository structure:

```text
upid/
|-- src/
|   |-- upid.h
|   `-- upid.cpp
|-- examples/
|   |-- arduino/
|   `-- esp_idf/
|-- tests/
|-- docs/
|-- library.properties
|-- CMakeLists.txt
`-- README.md
```

`src/upid.h` will expose the public API. `src/upid.cpp` will contain the
algorithm and state transitions. Arduino will consume the root
`library.properties`; ESP-IDF will consume the root `CMakeLists.txt`.

The Arduino metadata will declare `architectures=*` because the source is
architecture-neutral. This declaration means source-level compile portability,
not verified hardware support. Version 1 compilation checks cover representative
AVR, SAMD, RP2040, and ESP32 targets; hardware support is claimed only after
physical-board validation. ESP-IDF will register the implementation as a
dependency-free component and expose `src/` as its public include directory.

## Public API

The primary interface will follow this shape:

```cpp
upid::Config config;
config.kp = 1.0f;
config.ki = 0.2f;
config.kd = 0.05f;
config.outputMin = 0.0f;
config.outputMax = 255.0f;
config.derivativeFilterTau = 0.02f;
config.direction = upid::Direction::Direct;

upid::Controller controller(config);

upid::Result result =
    controller.update(setpoint, measurement, dtSeconds);
```

The exact constructor and setter signatures will be fixed in the
implementation plan, while preserving these semantics:

- `Config` contains gains, output limits, derivative-filter time constant, and
  controller direction.
- `Result` contains a `Status` and output value.
- `update(setpoint, measurement, dtSeconds)` performs one automatic control
  step.
- `setMode(Mode::Manual, manualOutput)` enters manual mode at a validated
  output.
- `setMode(Mode::Automatic)` marks a pending automatic transition.
- `reset(measurement, output)` establishes a known state without producing a
  control step.
- Validated setters support runtime tuning without partially applying invalid
  configuration.

The library uses `float` throughout version 1. PID gains use continuous-time
units: `dt` is seconds, `Ki` is per second, and `Kd` is seconds. Gains are
non-negative; `Direction::Direct` or `Direction::Reverse` determines the
controller action.

## Update Data Flow

```text
Inputs
  |
  v
Validate setpoint, measurement, dt, and configuration
  |
  +-- invalid --> return status + previous output; preserve all state
  |
  v
Compute signed error
  |
  +--> proportional term
  |
  +--> filtered derivative of measurement
  |
  `--> conditional integral candidate
             |
             v
       combine P + I - D
             |
             v
       apply output limits
             |
             v
       accept/reject integral candidate
             |
             v
       commit state and return output
```

Derivative-on-measurement avoids a derivative kick when the setpoint changes.
The derivative will use a first-order low-pass filter whose time constant is
specified in seconds. A zero time constant disables filtering.

Conditional integration will prevent accumulation when the output is saturated
and the current error would drive it farther into saturation. Integration may
continue when it moves the controller back toward the valid output range.

## Modes and State

```text
        setMode(Manual, output)
   +------------------------------+
   |                              v
+--+-------+                 +----+---+
| Automatic|                 | Manual |
+--+-------+                 +----+---+
   ^                              |
   +------------------------------+
        setMode(Automatic)
        initialize state for
        bumpless transfer
```

Manual mode returns the selected manual output and does not run an automatic
control step. `setMode(Mode::Automatic)` validates the existing configuration
and marks the transition as pending, but it does not guess a measurement. The
first valid `update(setpoint, measurement, dtSeconds)` initializes the previous
measurement, filtered derivative, and integral contribution from the current
manual output, returns that manual output, and commits automatic mode. Automatic
PID calculation starts on the following valid update. An invalid first update
returns its error status, preserves the manual output and all state, and leaves
the transition pending.

`reset()` initializes the prior measurement, integral contribution, filtered
derivative, and last output consistently. The implementation plan will define
the precise state initialization formula and test it explicitly.

## Validation and Fault Contract

The library will not throw exceptions or assert as part of its runtime
contract.

- `dtSeconds` must be finite and greater than zero.
- Setpoint and measurement must be finite.
- Gains, limits, filter time, and manual output must be finite.
- Gains and filter time must be non-negative.
- Output minimum must be strictly less than output maximum.
- Outputs are always clamped to configured limits.
- Invalid updates return a specific status and the previous output.
- Invalid updates do not change controller state.
- Invalid setters leave the last valid configuration unchanged.

The initial status set will include:

- `Ok`
- `InvalidTimeStep`
- `NonFiniteInput`
- `InvalidConfiguration`
- `WrongMode`

These five statuses are the complete version 1 status surface. A new status may
be considered only in a later design revision backed by an acceptance case that
cannot be represented correctly by this set.

## Testing and Acceptance

Host-side deterministic tests will cover:

- P, I, D, PI, and PID responses against known sequences
- Variable elapsed time
- Output saturation and conditional anti-windup recovery
- Derivative-on-measurement without setpoint kick
- Enabled and disabled derivative filtering
- Direct and reverse action
- Manual and automatic operation
- Bumpless transfer
- Reset behavior
- Valid and invalid runtime tuning
- NaN, infinity, invalid limits, and invalid elapsed time
- Long-running state behavior within `float` precision

Compilation checks will cover:

- Native host tests
- Representative Arduino AVR, SAMD, RP2040, and ESP32 targets
- A native ESP-IDF example and component

Compilation does not establish hardware correctness. Runtime behavior will be
reported as unverified until examples are exercised on physical boards.

## Beginner-Friendly Documentation

The root README will teach the library in this order:

1. Explain feedback control using a simple setpoint/measurement/output diagram.
2. Show a minimal controller example before presenting full configuration.
3. Explain P, I, and D with separate diagrams and plain-language effects.
4. Show the update loop and make the seconds-based `dt` contract prominent.
5. Provide Arduino and native ESP-IDF installation and usage examples.
6. Explain output limits, anti-windup, derivative filtering, direction, and
   manual mode using short examples.
7. Provide a practical manual-tuning flowchart.
8. Document status codes and troubleshooting.
9. Clearly separate compilation validation from physical-board validation.

GitHub-rendered Mermaid diagrams will be preferred for flows and relationships.
Small ASCII diagrams will be retained where they remain readable outside
GitHub. Graphics will clarify concepts rather than replace required API details.

## Non-Goals

Version 1 will not:

- Read a hardware or framework clock
- Schedule controller updates
- Allocate memory dynamically
- Depend on platform-specific headers
- Provide an RTOS task or interrupt wrapper
- Include autotuning, cascade control, feed-forward, or setpoint weighting
- Claim hard-real-time or hardware validation without measurements

## Rejected Simpler Version 1

An automatic-only controller with immutable configuration was considered and
rejected. Manual mode is required for commissioning, actuator override, and
safe handoff in typical embedded control systems. Validated runtime tuning is
required because gains and limits commonly need adjustment without reconstructing
application-owned controller objects. The selected API keeps both capabilities
without platform adapters, allocation, callbacks, or dynamic polymorphism.

