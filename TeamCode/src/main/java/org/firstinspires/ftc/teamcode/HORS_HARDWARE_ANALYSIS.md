# HORS Robot — Complete Hardware & Code Analysis

## Overview

This document analyzes the **MAINHORS26** monolithic `LinearOpMode` architecture
(`sushilm20/MAINHORS26`, branch `master`) and maps it to the **Simply-Complex-Decode**
FTCLib Command-Based architecture for future conversion.

---

## 1. MAINHORS26 Architecture (Source)

### Structure
```
teleop/
  OfficialHORS.java          ← Main TeleOp (monolithic LinearOpMode, ~400 lines)
  CameraTracking.java        ← Camera-based tracking TeleOp
  DriveStreamPoseTeleOp.java ← Drive+stream pose TeleOp
  PinpointAimTeleOp.java     ← Pinpoint-based aiming TeleOp
  AdaptiveHORS/              ← Adaptive flywheel variant
  experimentals/             ← Experimental TeleOps

subsystems/
  DriveController.java       ← Mecanum mixing + normalization
  FlywheelController.java    ← Dual-motor PIDF flywheel (close/far coefficient switching)
  GateController.java        ← Gate servo + intake motor auto-sequence with LED feedback
  ClawController.java        ← Timed open/close claw servo
  HoodController.java        ← Dual hood servos with nudge/preset control
  AutoShooter/               ← Autonomous shooter helpers

tracking/
  TurretController.java      ← IMU-based heading-hold turret with PID, homing sweep, freeze mode
  TurretGoalAimer.java       ← Goal-aiming turret variant
  PinpointTurretAimer.java   ← Pinpoint-based turret aiming
  CameraStreamManager.java   ← Camera stream management

extras/
  TelemetryData.java         ← Centralized telemetry display
  TelemetryHub.java          ← Panels telemetry hub
  ShooterMirror.java         ← Mirror primary shooter power to secondary motor
  RumbleHelper.java          ← Gamepad rumble utility
```

### Key Pattern: Monolithic OpMode
- `OfficialHORS.java` is a single `LinearOpMode` that:
  - Initializes all hardware directly in `runOpMode()`
  - Creates controller instances (DriveController, FlywheelController, etc.)
  - Runs a `while (opModeIsActive())` loop
  - Manually handles all gamepad input with rising-edge boolean tracking
  - Calls `controller.update()` methods each loop iteration

---

## 2. Simply-Complex-Decode Architecture (Target Pattern)

### Structure
```
robot/
  Robot.java                          ← Central robot class (hardware init + subsystem orchestration)
  subsystems/
    Shooter.java                      ← FTCLib Subsystem (implements Subsystem interface)
    Turret.java                       ← FTCLib Subsystem
    Intake.java                       ← FTCLib Subsystem
    Indexer.java                       ← FTCLib Subsystem
    Blocker.java                      ← FTCLib Subsystem
    Kicker.java                       ← FTCLib Subsystem
    Lights.java                       ← FTCLib Subsystem
    Camera.java                       ← Vision subsystem
    DistanceSensor.java               ← Sensor subsystem
    LimelightCamera.java              ← Limelight vision subsystem
  commands/
    subsystemcommands/                ← Individual subsystem state-change commands
      ShooterCommand.java
      TurretCommand.java
      IntakeCommand.java
      BlockerCommand.java
      IndexerCommand.java
    botcommands/                      ← Composite multi-subsystem commands
      TransferCommand.java            ← Full shooting sequence
      TransferCancelCommand.java
      FollowPathCommand.java
  calculations/
    ShooterCalculations.java          ← Math for hood angle + RPM from distance
    ShooterInput.java                 ← Data class for shooter inputs

opmodes/
  prod/                               ← Production TeleOp and Auto modes
    TeleopBlue.java                   ← Command-based teleop (blue alliance)
    TeleopRed.java                    ← Command-based teleop (red alliance)
    CloseSideAuto.java                ← Autonomous routines
    FarSideAuto.java
    ...
  testers/                            ← Test/debug OpModes

utils/
  constants/                          ← All tuning constants
    ShooterConstants.java
    TurretConstants.java
    BotConstants.java
    IntakeConstants.java
    IndexerConstants.java
    BlockerConstants.java
    KickerConstants.java
    LedConstants.java
    CameraConstants.java
    DistanceConstants.java
    ShooterMathConstants.java
    auto/                             ← Auto-specific path constants
  MyTelem.java                        ← Custom telemetry
  PoseParameters.java                 ← Pose configuration
  Prism/                              ← LED control library
```

### Key Pattern: FTCLib Command-Based
- **Subsystems** implement `Subsystem` interface with state enums
- **Commands** wrap state changes (e.g., `ShooterCommand` sets `Shooter.State`)
- **Composite commands** chain subsystem operations (e.g., `TransferCommand`)
- **OpModes** use `CommandScheduler` and `GamepadEx` for button→command bindings
- **Robot.java** centralizes hardware init and subsystem wiring

---

## 3. Complete HORS Hardware Inventory

### Motors (8 DcMotor ports)

| Name | Config String | Type | Direction | RunMode | ZeroPowerBehavior |
|---|---|---|---|---|---|
| Front Left Drive | `"frontLeft"` | DcMotor | FORWARD | default | default |
| Back Left Drive | `"backLeft"` | DcMotor | FORWARD | default | default |
| Front Right Drive | `"frontRight"` | DcMotor | REVERSE | default | default |
| Back Right Drive | `"backRight"` | DcMotor | REVERSE | default | default |
| Shooter (primary) | `"shooter"` | DcMotorEx | REVERSE | RUN_WITHOUT_ENCODER | FLOAT |
| Shooter 2 (mirror) | `"shooter2"` | DcMotor | FORWARD | RUN_WITHOUT_ENCODER | FLOAT |
| Turret | `"turret"` | DcMotor | FORWARD | RUN_USING_ENCODER | BRAKE |
| Intake Motor | `"intakeMotor"` | DcMotor | FORWARD | default | default |

### Servos (4 total)

| Name | Config String | Initial Position | Positions |
|---|---|---|---|
| Claw Servo | `"clawServo"` | 0.62 (open) | Open=0.62, Closed=0.3 |
| Left Hood Servo | `"leftHoodServo"` | 0.12 (min) | Min=0.12, Max=0.45 |
| Right Hood Servo | `"rightHoodServo"` | 0.16 (close) | Close=0.16, Far=0.26 |
| Gate Servo | `"gateServo"` | 0.485 (closed) | Open=0.67, Closed=0.485 |

### Sensors & Extras

| Name | Config String | Type | Notes |
|---|---|---|---|
| Pinpoint Odometry | `"pinpoint"` | GoBildaPinpointDriver | Primary IMU; call resetPosAndIMU() on init |
| Turret Limit Switch | `"turret_limit"` | DigitalChannel | Optional, active-low |
| LED 1 Red | `"led_1_red"` | LED | Optional |
| LED 1 Green | `"led_1_green"` | LED | Optional |
| LED 2 Red | `"led_2_red"` | LED | Optional |
| LED 2 Green | `"led_2_green"` | LED | Optional |
| Battery Voltage | (auto-detected) | VoltageSensor | First available via iterator |

### External Systems

| System | Usage |
|---|---|
| PedroPathing Follower | Pose tracking; `Constants.createFollower(hardwareMap)` |
| Panels Telemetry | `PanelsTelemetry.INSTANCE.getTelemetry()` |
| LynxModule Bulk Read | `BulkCachingMode.AUTO` on all hubs |

---

## 4. Constants Mapping

### MAINHORS26 → Simply-Complex-Decode Mapping

#### Flywheel / Shooter Constants

| MAINHORS26 Constant | Value | SC-Decode Equivalent | Notes |
|---|---|---|---|
| `CLOSE_kP` | 0.00146 | `ShooterConstants.kp` (100000) | Different PID approach |
| `CLOSE_kI` | 0.0027 | — | Not used in SC-Decode |
| `CLOSE_kD` | 0.00002 | `ShooterConstants.kd` (0) | |
| `CLOSE_kF` | 1.72 | `ShooterConstants.kf` (0.000175) | |
| `FAR_kP` | 0.00158 | — | HORS has dual PIDF modes |
| `FAR_kI` | 0.0040 | — | |
| `FAR_kD` | 0.00001 | — | |
| `FAR_kF` | 1.92 | — | |
| `TICKS_PER_REV` | 28.0 | `ShooterConstants.TICKS_PER_REV` (28.0) | Same |
| `closeRPM` | 2600 | `ShooterConstants.CLOSE_RPM` (4100) | Different RPM targets |
| `farRPM` | 3390 | — | HORS-specific |
| `RPM_SWITCH_THRESHOLD` | 3000.0 | — | HORS-specific dual-mode |
| `rpmTolerance` | 45.0 | `ShooterConstants.rpm_tolerance` | |

#### Hood Constants

| MAINHORS26 Constant | Value | SC-Decode Equivalent | Notes |
|---|---|---|---|
| `HOOD_MIN` | 0.12 | `ShooterConstants.OPEN_HOOD` (0.17) | Different range |
| `HOOD_MAX` | 0.45 | `ShooterConstants.CLOSE_HOOD` (0.32) | Different range |
| `RIGHT_HOOD_CLOSE` | 0.16 | — | HORS has dual hood servos |
| `RIGHT_HOOD_FAR` | 0.26 | — | |
| `HOOD_LEFT_STEP` | 0.025 | — | Manual nudge step |
| `HOOD_RIGHT_STEP` | 0.01 | — | |
| `HOOD_DEBOUNCE_MS` | 120 | — | |

#### Claw Constants

| MAINHORS26 Constant | Value | SC-Decode Equivalent | Notes |
|---|---|---|---|
| `CLAW_OPEN` | 0.62 | — | HORS-specific (no claw in SC-Decode) |
| `CLAW_CLOSED` | 0.3 | — | |
| `CLAW_CLOSE_MS` | 400 | — | |

#### Gate / Intake Constants

| MAINHORS26 Constant | Value | SC-Decode Equivalent | Notes |
|---|---|---|---|
| `GATE_OPEN` | 0.67 | `BlockerConstants.UNBLOCKED` | Similar concept |
| `GATE_CLOSED` | 0.485 | `BlockerConstants.BLOCKED` | |
| `INTAKE_DURATION_MS` | 1050 | — | Auto-sequence timing |
| `CLAW_TRIGGER_BEFORE_END_MS` | 400 | — | |
| `INTAKE_SEQUENCE_POWER` | 1.0 | `IntakeConstants.FORWARD` (1.0) | Same |

#### Turret Constants

| MAINHORS26 Constant | Value | SC-Decode Equivalent | Notes |
|---|---|---|---|
| `TURRET_KP` | 1.0 | `TurretConstants.P` (0.08) | Very different PID |
| `TURRET_KI` | 0.0 | — | |
| `TURRET_KD` | 0.235 | — | |
| `TURRET_MAX_POWER` | 1.0 | — | |
| `FF_GAIN` | 5.0 | — | HORS-specific feedforward |
| `TURRET_MIN_POS` | -1000 | — | Encoder-based (HORS uses motor) |
| `TURRET_MAX_POS` | 1000 | — | SC-Decode uses servos |
| `TICKS_PER_RADIAN_SCALE` | 0.87 | `TurretConstants.SLOPE` (-0.00244) | Different math |
| `HOMING_AMPLITUDE_TICKS` | 300 | — | HORS-specific homing |
| `HOMING_POWER` | 0.5 | — | |
| `HOMING_TIMEOUT_MS` | 3000 | — | |

---

## 5. Subsystem Mapping: MAINHORS26 → Simply-Complex-Decode

| MAINHORS26 Class | SC-Decode Equivalent | Key Differences |
|---|---|---|
| `DriveController` | Robot.java (inline) | SC-Decode uses Pedro Pathing follower for drive |
| `FlywheelController` | `Shooter` subsystem | HORS: dual PIDF modes (close/far), mirror motor, voltage-compensated FF. SC-Decode: single PID, state-based |
| `GateController` | `Blocker` subsystem + `Intake` subsystem | HORS combines gate+intake+LED in one controller. SC-Decode separates them |
| `ClawController` | — (no equivalent) | HORS-specific timed claw mechanism |
| `HoodController` | `Shooter.setHood()` | HORS: dual hood servos with nudge. SC-Decode: single hood servo |
| `TurretController` | `Turret` subsystem | HORS: DC motor encoder-based with PID + homing. SC-Decode: dual servos with angle math |
| `TelemetryData` | `MyTelem` + inline telemetry | Different telemetry approaches |
| `ShooterMirror` | — | HORS mirrors primary→secondary motor power |
| `RumbleHelper` | — | Gamepad rumble utility |

---

## 6. Gamepad Mapping Analysis

### MAINHORS26 Gamepad 1 (Driver)

| Input | Action | Rising-Edge? | SC-Decode Pattern |
|---|---|---|---|
| `left_stick_y` (inverted) | Drive axial | Continuous | Same concept |
| `left_stick_x` | Drive lateral | Continuous | Same concept |
| `right_stick_x` | Drive yaw | Continuous | Same concept |
| `touchpad` toggle | Toggle far/close mode | Rising-edge | `GamepadEx.getGamepadButton().whenPressed()` |
| `a` rising-edge | Reset turret encoder | Rising-edge | `whenPressed(new InstantCommand(...))` |
| `b` rising-edge | Toggle gate | Rising-edge | `whenPressed(new GateToggleCommand())` |
| `x` rising-edge | Manual claw trigger | Rising-edge | `whenPressed(new ClawCommand())` |
| `y` rising-edge | Start intake sequence | Rising-edge | `whenPressed(new IntakeSequenceCommand())` |
| `dpad_down` rising-edge | Toggle shooter on/off | Rising-edge | `whenPressed(new ShooterToggleCommand())` |
| `dpad_left` rising-edge | RPM -50 | Rising-edge | `whenPressed(new AdjustRPMCommand(-50))` |
| `dpad_right` rising-edge | RPM +50 | Rising-edge | `whenPressed(new AdjustRPMCommand(50))` |
| `dpad_up` | Turret homing sweep | Rising-edge | `whenPressed(new TurretHomingCommand())` |
| `right_bumper` held | Manual turret right | Held | `whileHeld(new ManualTurretCommand(0.35))` |
| `left_bumper` held | Manual turret left | Held | `whileHeld(new ManualTurretCommand(-0.35))` |
| `left_trigger > 0.1` | Intake reverse / flywheel | Held | Trigger-based command |
| `right_trigger > 0.1` | Intake forward | Held | Trigger-based command |
| `back` held | Flywheel calibration | Held | Flag-based |
| `a` held (NOT rising) | Hood nudge left up | Debounced | RepeatCommand or custom |
| `b` held (NOT rising) | Hood nudge left down | Debounced | RepeatCommand or custom |

### MAINHORS26 Gamepad 2 (Operator)
Shares most actions with Gamepad 1 — in SC-Decode pattern, use `GamepadEx` for both gamepads and bind same commands.

---

## 7. Key Architectural Differences

### 1. Turret: Motor vs Servos
- **HORS**: DC motor with encoder, PID heading-hold, homing sweep, freeze mode
- **SC-Decode**: Dual servos with calibrated angle-to-position math
- **Conversion impact**: Need new subsystem type (motor-based turret instead of servo-based)

### 2. Shooter: Dual PIDF vs Single PID
- **HORS**: `FlywheelController` with close/far coefficient switching, voltage-compensated feedforward, automatic PIDF mode selection based on RPM threshold
- **SC-Decode**: `Shooter` with single PID, state-based RPM targets
- **Conversion impact**: Preserve dual PIDF logic in new `FlywheelSubsystem`

### 3. Gate/Claw vs Blocker/Indexer
- **HORS**: `GateController` (gate servo + intake motor + LED auto-sequence), `ClawController` (timed close/open)
- **SC-Decode**: `Blocker` (gate servo), `Indexer` (ring staging), `Kicker` (ejection)
- **Conversion impact**: New `GateSubsystem` and `ClawSubsystem` needed

### 4. Hood: Dual Servos vs Single Servo
- **HORS**: `HoodController` with left + right hood servos, nudge controls, preset switching
- **SC-Decode**: Single hood servo integrated into `Shooter`
- **Conversion impact**: New `HoodSubsystem` with dual servo support

### 5. LED Integration
- **HORS**: Individual LED channels controlled by `GateController` (per-channel on/off)
- **SC-Decode**: `Lights` subsystem using GoBilda Prism LED strip
- **Conversion impact**: New LED handling approach

### 6. No Linear Indexer in HORS
- **HORS** does NOT have a linear indexer — omit `Indexer` subsystem from conversion
- The `GateController` + `ClawController` replace the indexer functionality

---

## 8. Conversion Readiness Summary

### New Subsystems Needed (FTCLib Command-Based)
1. **DriveSubsystem** — Mecanum drive (from `DriveController`)
2. **FlywheelSubsystem** — Dual-motor PIDF flywheel (from `FlywheelController`)
3. **GateSubsystem** — Gate servo + intake auto-sequence (from `GateController`)
4. **ClawSubsystem** — Timed claw servo (from `ClawController`)
5. **HoodSubsystem** — Dual hood servos (from `HoodController`)
6. **TurretSubsystem** — Motor-based PID turret (from `TurretController`)
7. **IntakeSubsystem** — Intake motor (currently inline in `OfficialHORS`)
8. **LEDSubsystem** — Individual LED channels (from `GateController` LED logic)

### New Commands Needed
1. **DefaultDriveCommand** — Continuous mecanum drive from gamepad
2. **ToggleShooterCommand** — Toggle flywheel on/off
3. **AdjustRPMCommand** — Increment/decrement target RPM
4. **ToggleFarCloseCommand** — Switch flywheel mode + hood preset
5. **GateToggleCommand** — Toggle gate open/close
6. **IntakeSequenceCommand** — Auto gate+intake+claw sequence
7. **ManualClawCommand** — Trigger claw close/open cycle
8. **TurretHomingCommand** — Initiate homing sweep
9. **ManualTurretCommand** — Manual turret power while held
10. **HoodNudgeCommand** — Debounced hood adjustment
11. **IntakeForwardCommand** / **IntakeReverseCommand** — Trigger-based intake

### Constants to Create
- `FlywheelConstants.java` — All PIDF coefficients (close + far), RPM targets, thresholds
- `GateConstants.java` — Gate positions, intake timing, sequence parameters
- `ClawConstants.java` — Claw positions, timing
- `HoodConstants.java` — Hood servo range, steps, debounce
- `TurretConstants.java` — PID gains, limits, homing parameters, feedforward
- `DriveConstants.java` — Motor directions (if needed)
- `LEDConstants.java` — LED channel names

---

## 9. Files from MAINHORS26 to Preserve Logic From

| Source File | Size | Priority | Notes |
|---|---|---|---|
| `OfficialHORS.java` | 15.4 KB | HIGH | Main control flow, gamepad mapping |
| `FlywheelController.java` | 16.3 KB | HIGH | Complex dual PIDF with voltage compensation |
| `TurretController.java` | 24.6 KB | HIGH | PID heading-hold, homing sweep, freeze mode |
| `GateController.java` | 3.6 KB | MEDIUM | Gate + intake auto-sequence with LED |
| `DriveController.java` | 2.7 KB | LOW | Simple mecanum mixing |
| `HoodController.java` | 2.4 KB | MEDIUM | Dual hood with nudge controls |
| `ClawController.java` | 1.0 KB | LOW | Simple timed servo |
| `TelemetryData.java` | 3.0 KB | LOW | Telemetry formatting |
| `ShooterMirror.java` | 0.5 KB | LOW | Mirror motor power |
| `RumbleHelper.java` | 0.4 KB | LOW | Gamepad rumble utility |
