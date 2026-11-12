# MotorDriverCard-ApplicationModule

Provides an ApplicationCore interface to the MotorDriverCard library

## Logging

The module writes diagnostic information through the ChimeraTK logger
(`ChimeraTK::logger()`). The minimum severity is controlled by the
`CHIMERATK_LOG_LEVEL` environment variable, which accepts `trace`, `debug`,
`info`, `warning` and `error`. The default is `info`.

```text
CHIMERATK_LOG_LEVEL=debug ./your-application
```

Each log line starts with a time stamp, the logging context and the severity.
The modules use the log helpers of `ApplicationModule` (`info()`, `warning()`,
`error()`, `debug()`, `trace()`), so the context is the class name of the
module:

| Context | Contents |
| --- | --- |
| `ControlInputHandler` | Every accepted and every rejected control command |
| `ReadbackHandler` | Opening the motor, SPI errors, entering the FAULT state |
| `StatusModule` | Changes of the motor state and the notification message |
| `StepperMotorModule` | Initialisation of the motor proxy device |
| `Motor` | Creation and closing of the motor objects |

The contexts `StepperMotorModule` and `Motor` are passed explicitly to
`ChimeraTK::logger()`, since `StepperMotorModule` is a `ModuleGroup` and
`Motor` is a plain class which provide no log helpers.

The default `info` level already reports every command that was rejected,
i.e. every reason why the motor will not move. Such messages contain the
phrase `was not executed, motor will not move` followed by the cause, for
example:

```text
ControlInputHandler [E] Setting target position to 500 steps was not
executed, motor will not move: System not calibrated
```

Rejected commands are reported with `error` severity for the following
causes: the motor device is in recovery, the motor driver card returned a
non-success exit status, the motor is not in the idle state required to
start a movement or a calibration, or the motor driver card raised an
exception. Repetitive SPI readout failures are reported with `warning`
severity until the motor is closed and the module enters recovery, which is
reported with `error` severity.

Use `trace` to additionally see which command is currently being dispatched
and the values read back in each readout cycle.
