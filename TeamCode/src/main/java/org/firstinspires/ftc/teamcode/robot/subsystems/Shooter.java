package org.firstinspires.ftc.teamcode.robot.subsystems;

import static org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants.hoodServoPosition;
import static org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants.RPM_OFFSET;
import static org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants.startingVelocity;
import static org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants.tuningTestingRPM;
import static org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants.currentVelocity;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Vector;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.MyTelem;
import org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.utils.constants.ShooterMathConstants;

public class Shooter implements Subsystem {
    DcMotorEx shooterMotor, shooterMotor2;
    Servo hoodServo;
    PIDController shooterRPMPID;

    public ShooterState state = ShooterState.STOP;
    public Shooter(DcMotorEx shooterMotor, DcMotorEx shooterMotor2, Servo hoodServo){
        this.hoodServo = hoodServo;
        shooterRPMPID = new PIDController(ShooterConstants.kp, ShooterConstants.ki, ShooterConstants.kd);
        shooterRPMPID.setTolerance(10);
        this.shooterMotor = shooterMotor;
        this.shooterMotor2 = shooterMotor2;
    }

    public void setState(ShooterState state){
        this.state = state;
        switch (state) {
            case CLOSE:
                currentVelocity = ShooterConstants.closeShootRPM;
                hoodServoPosition = ShooterConstants.closeHoodAngle;
                break;
            case STOP:
                currentVelocity = startingVelocity;
                hoodServoPosition = ShooterConstants.openAngle;
                break;
            case TESTING:
                currentVelocity = tuningTestingRPM;
                hoodServoPosition = ShooterConstants.tuningTestingHoodPosition;
                break;
            case SPEEDING_UP:
                currentVelocity = ShooterConstants.speedingVelocity;
                break;
            case MATH:
                double hoodAngle = Robot.getHoodAngle();
                double hoodPos = setHood(hoodAngle);
                double rpm = getRPM(Robot.getShooterMathRPM(), hoodPos);
                setHood(hoodAngle);
                currentVelocity = rpm;
                MyTelem.addData("HOOD ANGLE", hoodAngle);
                MyTelem.addData("HOOD POSITION", hoodServoPosition);
                MyTelem.addData("RPM", rpm);
                break;
        }
    }
    public double setHood(double angleRad) {
        double minAngle = ShooterMathConstants.HOOD_MIN_ANGLE; //0.32
        double maxAngle = ShooterMathConstants.HOOD_MAX_ANGLE; //0.17
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
    public void setShooterPIDPower(double targetRPM){
        double topVelocity = Math.abs(shooterMotor.getVelocity());
        double currentRPM = (ticksPerSecToRPM(topVelocity));

        shooterRPMPID.setPID(ShooterConstants.kp, ShooterConstants.ki, ShooterConstants.kd);

        double power = shooterRPMPID.calculate(currentRPM, targetRPM);
        power += (targetRPM > 0) ? (ShooterConstants.kf * (targetRPM / ShooterConstants.MAX_RPM)) : 0.0;
        power = Range.clip(power, 0, 1);

        double currentVoltage = Robot.voltage;
        shooterMotor.setPower(power * 13 / currentVoltage);
        shooterMotor2.setPower(power * 13 / currentVoltage);

        MyTelem.addData("Shooter Current RPM", currentRPM);
        MyTelem.addData("Shooter Target RPM", targetRPM);
    }
    public boolean shooterAtRPM(){
        return Math.abs(shooterRPMPID.getPositionError()) <= 200;
    }

    public boolean atRPM() {
        return shooterAtRPM();
    }

    public ShooterState getState(){
        return state;
    }

    @Override
    public void periodic() {
        setState(state);
        shooterRPMPID.setPID(ShooterConstants.kp, ShooterConstants.ki, ShooterConstants.kd);
        hoodServo.setPosition(hoodServoPosition);
        setShooterPIDPower(currentVelocity);
    }

    public enum ShooterState {
        CLOSE, STOP, TESTING, MATH, SPEEDING_UP
    }
}
