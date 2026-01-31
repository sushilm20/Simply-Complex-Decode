package org.firstinspires.ftc.teamcode.robot.subsystems;

import static org.firstinspires.ftc.teamcode.utils.constants.HoodConstants.hoodServoPosition;
import static org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants.RPM_OFFSET;
import static org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants.startingVelocity;
import static org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants.tuningTestingRPM;
import static org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants.currentVelocity;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Vector;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.MyTelem;
import org.firstinspires.ftc.teamcode.utils.constants.HoodConstants;
import org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants;

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
                hoodServoPosition = HoodConstants.closeHoodAngle;
                break;
            case STOP:
                currentVelocity = startingVelocity;
                hoodServoPosition = HoodConstants.openAngle;
                break;
            case TESTING:
                currentVelocity = tuningTestingRPM;
                hoodServoPosition = HoodConstants.tuningTestingHoodPosition;
                break;
            case SPEEDING_UP:
                currentVelocity = ShooterConstants.speedingVelocity;
                break;
            case MATH:
                double rpm = getRPM(Robot.getEffectiveCoordinates()) + RPM_OFFSET;
                currentVelocity = rpm;
                break;
        }
    }
    public double getRPM(Pose pose){
        Vector velocity = Robot.velocity;
        double distance = Robot.getDistanceFromGoal(pose);
        double speed = 0.0270551 * distance * distance + 2.69484 * distance + 3513.8641;
        MyTelem.addData("speed", speed);
        return speed;
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
        MyTelem.addData("Shooter Power", power * 13 / currentVoltage);
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
