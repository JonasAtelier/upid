# UPID API Refactor Completion Design

## Goal

Finish the in-progress public API refactor without changing the established PID
control behavior, then update every shipped consumer and user-facing description
of that API.

## Public API

- Rename the controller type from `controller` to `upid` as already started.
- Make `update(float target, float mea, float dt_s)` return `upid_sta`.
- Read the current actuator value with `get_output()` after a successful update.
- Remove `upid_res`, because the new API has no operation that produces it.
- Do not add a compatibility alias for `controller`; this refactor is an
  intentional source-breaking API change.

## Controller Behavior

The refactor must preserve the behavior of the implementation at commit
`0e14a10` except for the approved public API changes. In particular:

- The first automatic update initializes derivative state deterministically.
- A rejected integral candidate must not affect the emitted output; output is
  recomputed from the retained integral value.
- Invalid updates leave the previous output and controller state unchanged.
- Manual-to-automatic transition remains bumpless and holds the manual output
  for the handoff update.
- Existing validation, output clamping, reset, and runtime-retuning semantics
  remain unchanged.

## Consumers and Documentation

`examples/basic_pid.cpp` will construct `upid`, check the status returned by
`update()`, then retrieve and apply `pid.get_output()`.

`README.md` will describe the same status-plus-accessor flow everywhere,
including the quick start, update lifecycle, manual mode, troubleshooting, and
linker diagnostics. Terminology will use the new parameter names where it helps
clarity, while continuing to explain target, measurement, and elapsed seconds.

## Verification

- Compile with C++11, optimization, strict warnings, and `-Werror`.
- Run the host example and its convergence/output-bound assertions.
- Add focused regression coverage for deterministic first update, anti-windup,
  invalid-input state preservation, manual/automatic transition, reset, and
  runtime retuning.
- Run `git diff --check` before each completion claim.

## Commit Structure

1. Preserve controller behavior and complete the public API refactor.
2. Update the runnable example for the new API.
3. Update the README for the new API and verification scope.

Commits will use the repository's existing concise style and will not include
co-author trailers. After all verification succeeds, push `main` to `origin`.
