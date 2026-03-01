package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.MyTelem;
import org.firstinspires.ftc.teamcode.utils.constants.TurretConstants;

/**
 * HORS turret subsystem — DC motor with encoder, IMU-based heading-hold PID,
 * homing sweep, and freeze mode.
 */
public class Turret implements Subsystem {

    private final DcMotor turretMotor;
    private final GoBildaPinpointDriver pinpoint; // preferred heading source

    // PID state
    private double turretIntegral = 0.0;
    private int lastErrorTicks = 0;
    private long lastTimeMs = -1L;
    private double lastAppliedPower = 0.0;
    private double lastDerivative = 0.0;

    // Heading / encoder reference
    private double headingReferenceRad = 0.0;
    private int turretEncoderReference = 0;
    private int encoderOffset = 0;

    // Manual→auto transition
    private boolean manualActiveLast = false;

    // Homing sweep state
    private boolean homingMode = false;
    private boolean homingCommandPrev = false;
    private boolean homingDirectionPos = true;
    private int homingTarget = TurretConstants.HOMING_AMPLITUDE_TICKS;
    private long homingStartMs = 0L;

    // Freeze/hold mode
    private boolean freezeMode = false;
    private int freezeHoldTarget = 0;

    // State
    public TurretState state = TurretState.BACK;

    // Manual control
    private boolean manualActive = false;
    private double manualPower = 0.0;

    public Turret(DcMotor turretMotor, GoBildaPinpointDriver pinpoint) {
        this.turretMotor = turretMotor;
        this.pinpoint = pinpoint;
        captureReferences();
        resetPidState();
    }

    public void setState(TurretState state) {
        if (state == null) return;
        this.state = state;
    }

    public void setManualControl(boolean active, double power) {
        this.manualActive = active;
        this.manualPower = power;
    }

    public void commandHomingSweep(boolean command) {
        if (command && !homingCommandPrev) {
            homingMode = true;
            homingDirectionPos = true;
            homingTarget = TurretConstants.HOMING_AMPLITUDE_TICKS;
            homingStartMs = System.currentTimeMillis();
        }
        homingCommandPrev = command;
    }

    /**
     * Main update — call every loop iteration.
     */
    public void update() {
        int currentVirtualTicks = getVirtualEncoderPosition();

        // Homing sweep mode
        if (homingMode) {
            runHomingSweep();
            manualActiveLast = false;
            return;
        }

        // Freeze/hold mode (after homing)
        if (freezeMode) {
            int error = freezeHoldTarget - currentVirtualTicks;
            if (Math.abs(error) <= TurretConstants.SMALL_DEADBAND_TICKS) {
                turretMotor.setPower(0);
            } else {
                double p = TurretConstants.TURRET_KP * error;
                p = Range.clip(p, -TurretConstants.TURRET_MAX_POWER, TurretConstants.TURRET_MAX_POWER);
                turretMotor.setPower(p);
            }
            manualActiveLast = false;
            return;
        }

        // Manual control
        if (manualActive) {
            applyManualPower(manualPower, currentVirtualTicks);
            if (!manualActiveLast) {
                // Just entered manual
            }
            manualActiveLast = true;
            return;
        }

        // Transitioning from manual→auto: recapture heading reference
        if (manualActiveLast) {
            captureReferences();
            resetPidState();
            manualActiveLast = false;
        }

        // Auto heading-hold PID
        switch (state) {
            case BACK:
                // Hold center (0 offset from reference)
                runHeadingHoldPID(currentVirtualTicks);
                break;
            case MATH:
                // Use math-based aiming through heading-hold
                runHeadingHoldPID(currentVirtualTicks);
                break;
        }
    }

    private void runHeadingHoldPID(int currentVirtualTicks) {
        long nowMs = System.currentTimeMillis();
        if (lastTimeMs < 0) lastTimeMs = nowMs;
        double dtSec = (nowMs - lastTimeMs) / 1000.0;
        lastTimeMs = nowMs;
        if (dtSec <= 0 || dtSec > 1.0) dtSec = 0.02;

        // Compute desired encoder ticks from heading change
        double currentHeading = getHeadingRadians();
        double headingDelta = normalizeAngle(currentHeading - headingReferenceRad);
        int desiredTicks = turretEncoderReference
                + (int)(headingDelta * TurretConstants.TICKS_PER_RADIAN_SCALE
                * (TurretConstants.TURRET_MAX_POS - TurretConstants.TURRET_MIN_POS) / (2.0 * Math.PI));

        // Clamp to limits
        desiredTicks = Range.clip(desiredTicks, TurretConstants.TURRET_MIN_POS, TurretConstants.TURRET_MAX_POS);

        int errorTicks = desiredTicks - currentVirtualTicks;

        // Rightward asymmetry damping
        if (errorTicks > 0 && Math.abs(errorTicks) < TurretConstants.RIGHTWARD_DAMP_ERROR_WINDOW) {
            errorTicks = (int)(errorTicks * TurretConstants.RIGHTWARD_ENCODER_DAMP);
        }

        // Deadband
        if (Math.abs(errorTicks) <= TurretConstants.SMALL_DEADBAND_TICKS) {
            turretMotor.setPower(0);
            lastAppliedPower = 0;
            return;
        }

        // PID
        turretIntegral += errorTicks * dtSec;
        turretIntegral = Range.clip(turretIntegral, -TurretConstants.INTEGRAL_CLAMP, TurretConstants.INTEGRAL_CLAMP);

        double rawDeriv = (dtSec > 0) ? (errorTicks - lastErrorTicks) / dtSec : 0;
        lastDerivative = TurretConstants.DERIV_FILTER_ALPHA * lastDerivative
                + (1.0 - TurretConstants.DERIV_FILTER_ALPHA) * rawDeriv;

        double pidOut = TurretConstants.TURRET_KP * errorTicks
                + TurretConstants.TURRET_KI * turretIntegral
                + TurretConstants.TURRET_KD * lastDerivative;

        // Feedforward from heading angular velocity
        double angularVel = (dtSec > 0) ? headingDelta / dtSec : 0;
        double ff = TurretConstants.FF_GAIN * angularVel;

        double totalPower = pidOut + ff;
        totalPower = Range.clip(totalPower, -TurretConstants.TURRET_MAX_POWER, TurretConstants.TURRET_MAX_POWER);

        // Enforce hard limits
        if ((currentVirtualTicks >= TurretConstants.TURRET_MAX_POS && totalPower > 0)
                || (currentVirtualTicks <= TurretConstants.TURRET_MIN_POS && totalPower < 0)) {
            totalPower = 0;
        }

        // Smooth power
        double smoothed = TurretConstants.POWER_SMOOTH_ALPHA * lastAppliedPower
                + (1.0 - TurretConstants.POWER_SMOOTH_ALPHA) * totalPower;

        turretMotor.setPower(smoothed);
        lastAppliedPower = smoothed;
        lastErrorTicks = errorTicks;

        MyTelem.addData("turret.desired", desiredTicks);
        MyTelem.addData("turret.virtual", currentVirtualTicks);
        MyTelem.addData("turret.error", errorTicks);
    }

    private boolean runHomingSweep() {
        if (System.currentTimeMillis() - homingStartMs > TurretConstants.HOMING_TIMEOUT_MS) {
            turretMotor.setPower(0);
            homingMode = false;
            return true;
        }

        int current = getVirtualEncoderPosition();

        if (homingDirectionPos && current >= homingTarget - TurretConstants.HOMING_TARGET_DEADBAND) {
            homingDirectionPos = false;
            homingTarget = -TurretConstants.HOMING_AMPLITUDE_TICKS;
        } else if (!homingDirectionPos && current <= homingTarget + TurretConstants.HOMING_TARGET_DEADBAND) {
            homingDirectionPos = true;
            homingTarget = TurretConstants.HOMING_AMPLITUDE_TICKS;
        }

        double power = homingDirectionPos ? TurretConstants.HOMING_POWER : -TurretConstants.HOMING_POWER;

        if ((current >= TurretConstants.TURRET_MAX_POS && power > 0)
                || (current <= TurretConstants.TURRET_MIN_POS && power < 0)) {
            power = 0;
        }

        turretMotor.setPower(power);
        return false;
    }

    private void applyManualPower(double power, int currentVirtualTicks) {
        if ((currentVirtualTicks >= TurretConstants.TURRET_MAX_POS && power > 0)
                || (currentVirtualTicks <= TurretConstants.TURRET_MIN_POS && power < 0)) {
            power = 0;
        }
        power = Range.clip(power, -1.0, 1.0);
        turretMotor.setPower(power);
    }

    public void captureReferences() {
        headingReferenceRad = getHeadingRadians();
        turretEncoderReference = getVirtualEncoderPosition();
    }

    public void resetPidState() {
        turretIntegral = 0;
        lastErrorTicks = 0;
        lastTimeMs = -1L;
        lastAppliedPower = 0;
        lastDerivative = 0;
    }

    public void recenterAndResume(boolean resetEncoder) {
        if (resetEncoder) {
            try {
                turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } catch (Exception ignored) {}
            encoderOffset = 0;
        }
        captureReferences();
        resetPidState();
        freezeMode = false;
        homingMode = false;
    }

    public void disable() {
        turretMotor.setPower(0);
    }

    private int getVirtualEncoderPosition() {
        return turretMotor.getCurrentPosition() - encoderOffset;
    }

    private double getHeadingRadians() {
        if (pinpoint != null) {
            try {
                return -pinpoint.getHeading(AngleUnit.RADIANS);
            } catch (Exception ignored) {}
        }
        return 0.0;
    }

    private static double normalizeAngle(double angle) {
        while (angle <= -Math.PI) angle += 2.0 * Math.PI;
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        return angle;
    }

    public TurretState getState() {
        return state;
    }

    @Override
    public void periodic() {
        update();
    }

    public enum TurretState {
        BACK, MATH
    }
}
