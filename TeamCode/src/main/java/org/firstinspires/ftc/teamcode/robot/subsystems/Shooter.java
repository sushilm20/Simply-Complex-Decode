package org.firstinspires.ftc.teamcode.robot.subsystems;

import static org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants.hoodServoPosition;
import static org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants.RPM_OFFSET;
import static org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants.karthikstfu;
import static org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants.speedingVelocity;
import static org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants.startingVelocity;
import static org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants.tuningTestingRPM;
import static org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants.currentVelocity;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.MyTelem;
import org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.utils.constants.ShooterMathConstants;

public class Shooter implements Subsystem {
    DcMotorEx shooterMotor;   // primary (encoder)
    DcMotor shooterMotor2;    // mirror motor
    Servo leftHoodServo, rightHoodServo;

    // HORS PIDF internal state
    private double targetRpm = 0;
    private double lastError = 0.0;
    private double integralSum = 0.0;
    private double lastDerivativeEstimate = 0.0;
    private int lastPos = 0;
    private double currentRpm = 0.0;
    private double lastAppliedPower = 0.0;
    private boolean usingFarCoefficients = false;
    private long lastUpdateTimeMs = 0;

    // Hood positions
    private static final double HOOD_MIN = 0.12;
    private static final double HOOD_MAX = 0.45;
    private static final double RIGHT_HOOD_CLOSE = 0.16;
    private static final double RIGHT_HOOD_FAR = 0.26;

    private double leftHoodPos = HOOD_MIN;
    private double rightHoodPos = RIGHT_HOOD_CLOSE;

    public ShooterState state = ShooterState.STOP;

    public Shooter(DcMotorEx shooterMotor, DcMotor shooterMotor2,
                   Servo leftHoodServo, Servo rightHoodServo) {
        this.shooterMotor = shooterMotor;
        this.shooterMotor2 = shooterMotor2;
        this.leftHoodServo = leftHoodServo;
        this.rightHoodServo = rightHoodServo;
        lastPos = shooterMotor.getCurrentPosition();
        lastUpdateTimeMs = System.currentTimeMillis();
    }

    public void setState(ShooterState state){
        this.state = state;
        switch (state) {
            case CLOSE:
                targetRpm = ShooterConstants.closeShootRPM;
                rightHoodPos = RIGHT_HOOD_CLOSE;
                break;
            case STOP:
                targetRpm = startingVelocity;
                break;
            case TESTING:
                targetRpm = tuningTestingRPM;
                break;
            case SPEEDING_UP:
                targetRpm = ShooterConstants.speedingVelocity;
                break;
            case FAR:
                targetRpm = ShooterConstants.farShootRPM;
                rightHoodPos = RIGHT_HOOD_FAR;
                break;
            case MATH:
                double hoodAngle = Robot.getHoodAngle();
                double hoodPos = setHood(hoodAngle);
                double rpm = getRPM(Robot.getShooterMathRPM(), hoodPos);
                setHood(hoodAngle);
                targetRpm = rpm;
                MyTelem.addData("HOOD ANGLE", hoodAngle);
                MyTelem.addData("HOOD POSITION", hoodServoPosition);
                MyTelem.addData("RPM", rpm);
                break;
        }
        currentVelocity = targetRpm;
    }

    public double setHood(double angleRad) {
        double minAngle = ShooterMathConstants.HOOD_MIN_ANGLE;
        double maxAngle = ShooterMathConstants.HOOD_MAX_ANGLE;
        double servoHigh = 0.17;
        double servoLow = 0.32;
        double loA = Math.min(minAngle, maxAngle);
        double hiA = Math.max(minAngle, maxAngle);
        angleRad = MathFunctions.clamp(angleRad, loA, hiA);
        hoodServoPosition = servoLow + (angleRad - loA) * (servoHigh - servoLow) / (hiA - loA);
        return hoodServoPosition;
    }

    public double getRPM(double velocity, double hood){
        MyTelem.addData("VELOCITY", velocity);
        double hoodMultiplier = 2.79242 * hood * hood - 2.05502 * hood + 1.29534;
        double baseRPM = 24.27316 * velocity - 1943.16341;
        MyTelem.addData("HOOD MULTIPLIER", hoodMultiplier);
        MyTelem.addData("BASE RPM", baseRPM);
        return baseRPM * hoodMultiplier;
    }

    public double ticksPerSecToRPM(double tps){
        return tps * 60.0 / ShooterConstants.TICKS_PER_REV;
    }

    /**
     * HORS-style PIDF update with close/far coefficient switching and voltage-compensated FF.
     */
    private void updatePIDF() {
        long nowMs = System.currentTimeMillis();
        double dtSeconds = (nowMs - lastUpdateTimeMs) / 1000.0;
        lastUpdateTimeMs = nowMs;
        if (dtSeconds <= 0 || dtSeconds > 1.0) dtSeconds = 0.02;

        // Select coefficients based on target RPM
        usingFarCoefficients = (targetRpm >= ShooterConstants.RPM_SWITCH_THRESHOLD);
        double kP, kI, kD, kF, integralLimit, derivAlpha, rpmAlpha, powerAlpha, refVoltage, refMaxTPS;
        if (usingFarCoefficients) {
            kP = ShooterConstants.FAR_kP; kI = ShooterConstants.FAR_kI;
            kD = ShooterConstants.FAR_kD; kF = ShooterConstants.FAR_kF;
            integralLimit = ShooterConstants.FAR_integralLimit;
            derivAlpha = ShooterConstants.FAR_derivativeAlpha;
            rpmAlpha = ShooterConstants.FAR_rpmFilterAlpha;
            powerAlpha = ShooterConstants.FAR_powerSmoothingAlpha;
            refVoltage = ShooterConstants.FAR_ffReferenceVoltage;
            refMaxTPS = ShooterConstants.FAR_ffReferenceMaxTicksPerSec;
        } else {
            kP = ShooterConstants.CLOSE_kP; kI = ShooterConstants.CLOSE_kI;
            kD = ShooterConstants.CLOSE_kD; kF = ShooterConstants.CLOSE_kF;
            integralLimit = ShooterConstants.CLOSE_integralLimit;
            derivAlpha = ShooterConstants.CLOSE_derivativeAlpha;
            rpmAlpha = ShooterConstants.CLOSE_rpmFilterAlpha;
            powerAlpha = ShooterConstants.CLOSE_powerSmoothingAlpha;
            refVoltage = ShooterConstants.CLOSE_ffReferenceVoltage;
            refMaxTPS = ShooterConstants.CLOSE_ffReferenceMaxTicksPerSec;
        }

        // Measure current RPM
        double ticksPerSecond;
        try {
            ticksPerSecond = shooterMotor.getVelocity();
            lastPos = shooterMotor.getCurrentPosition();
        } catch (Exception e) {
            int pos = shooterMotor.getCurrentPosition();
            int delta = pos - lastPos;
            lastPos = pos;
            ticksPerSecond = delta / dtSeconds;
        }
        double rawRpm = (ticksPerSecond * 60.0) / ShooterConstants.TICKS_PER_REV;
        currentRpm = rpmAlpha * currentRpm + (1.0 - rpmAlpha) * rawRpm;

        if (targetRpm <= 0) {
            shooterMotor.setPower(0);
            if (shooterMotor2 != null) shooterMotor2.setPower(0);
            integralSum = 0;
            lastError = 0;
            lastAppliedPower = 0;
            return;
        }

        double error = targetRpm - currentRpm;

        // Integral with anti-windup
        integralSum += error * dtSeconds;
        if (integralSum > integralLimit) integralSum = integralLimit;
        if (integralSum < -integralLimit) integralSum = -integralLimit;

        // Filtered derivative
        double rawDeriv = (dtSeconds > 0) ? (error - lastError) / dtSeconds : 0;
        lastDerivativeEstimate = derivAlpha * lastDerivativeEstimate + (1.0 - derivAlpha) * rawDeriv;

        // PID output
        double pidOut = kP * error + kI * integralSum + kD * lastDerivativeEstimate;

        // Voltage-compensated feedforward
        double batteryV = Robot.voltage;
        if (batteryV < 1.0) batteryV = 12.0;
        double targetTPS = targetRpm * ShooterConstants.TICKS_PER_REV / 60.0;
        double ff = kF * (targetTPS / refMaxTPS) * (refVoltage / batteryV);

        double rawOut = pidOut + ff;
        rawOut = Range.clip(rawOut, 0, 1);

        // Power smoothing
        double smoothedOut = powerAlpha * lastAppliedPower + (1.0 - powerAlpha) * rawOut;
        smoothedOut = Range.clip(smoothedOut, 0, 1);

        shooterMotor.setPower(smoothedOut);
        if (shooterMotor2 != null) shooterMotor2.setPower(smoothedOut);

        lastAppliedPower = smoothedOut;
        lastError = error;

        MyTelem.addData("Shooter Current RPM", currentRpm);
        MyTelem.addData("Shooter Target RPM", targetRpm);
        MyTelem.addData("PIDF Mode", usingFarCoefficients ? "FAR" : "CLOSE");
    }

    public void setShooterPIDPower(double targetRPM){
        // Legacy method — just update targetRpm, actual PIDF runs in periodic()
        this.targetRpm = targetRPM;
    }

    public boolean shooterAtRPM(){
        double tolerance = usingFarCoefficients ?
                ShooterConstants.FAR_rpmTolerance : ShooterConstants.CLOSE_rpmTolerance;
        return Math.abs(targetRpm - currentRpm) <= tolerance;
    }

    public boolean atRPM() {
        return shooterAtRPM();
    }

    public boolean isUsingFarCoefficients() {
        return usingFarCoefficients;
    }

    public double getCurrentRPM() {
        return currentRpm;
    }

    public double getTargetRPM() {
        return targetRpm;
    }

    public ShooterState getState(){
        return state;
    }

    // Hood control methods
    public void setRightHoodPosition(double pos) {
        rightHoodPos = Range.clip(pos, HOOD_MIN, HOOD_MAX);
    }

    public void setLeftHoodPosition(double pos) {
        leftHoodPos = Range.clip(pos, HOOD_MIN, HOOD_MAX);
    }

    public double getLeftHoodPos() { return leftHoodPos; }
    public double getRightHoodPos() { return rightHoodPos; }

    @Override
    public void periodic() {
        setState(state);
        // Apply hood positions
        if (leftHoodServo != null) leftHoodServo.setPosition(leftHoodPos);
        if (rightHoodServo != null) rightHoodServo.setPosition(rightHoodPos);
        // Run PIDF controller
        updatePIDF();
    }

    public enum ShooterState {
        CLOSE, STOP, TESTING, MATH, SPEEDING_UP, FAR
    }
}
