package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.MyTelem;
import org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants;

@Config
public class Shooter implements Subsystem {
    DcMotorEx shooterMotor, /*bottomShooter,*/ counterRoller;
    Servo hood;
    PIDController shooterRPMPID, counterRPMPID;

    public ShooterState state = ShooterState.STOP;
    public Shooter(DcMotorEx shooterMotor, DcMotorEx counterRoller, Servo hood){
        shooterRPMPID = new PIDController(ShooterConstants.kp, ShooterConstants.ki, ShooterConstants.kd);
        shooterRPMPID.setTolerance(10);
        counterRPMPID = new PIDController(ShooterConstants.CRkp, ShooterConstants.CRki, ShooterConstants.CRkd);
        counterRPMPID.setTolerance(10);

        this.shooterMotor = shooterMotor;
        this.counterRoller = counterRoller;
        this.hood = hood;
    }

    public void setState(ShooterState state){
        this.state = state;
//        switch (state) {
//            case SHOOTING:
//                ShooterInput constants = ShooterCalculations.getHoodAndPower();
//                setPIDPower(constants.getMotorRPM());
//                setHood(constants.getHoodAngle());
//            case STOP:
//                setPIDPower(0);
//        }
        switch (state) {
            case CLOSE:
                setShooterPIDPower(ShooterConstants.closeShootRPM);
                setCounterRollerPIDPower(ShooterConstants.closeShootCounterRollerRPM);
                break;
            case FAR:
                setShooterPIDPower(ShooterConstants.farShootRPM);
                setCounterRollerPIDPower(ShooterConstants.farShootCounterRollerRPM);
                break;
            case STOP:
                setShooterPIDPower(0);
                setCounterRollerPIDPower(0);
                break;
            case TESTING:
                setShooterPIDPower(ShooterConstants.tuningTestingRPM);
                setCounterRollerPIDPower(ShooterConstants.tuningTestingCounterRollerRPM);
                break;
            case CLOSEAUTO:
                setShooterPIDPower(ShooterConstants.closeShootAutoRPM);
                break;
            case MATH:
                double rpm = getRPM();
                MyTelem.addData("Shooter RPM", rpm);
                setShooterPIDPower(rpm);
                setCounterRollerPIDPower(2.38 * rpm);
                break;
        }
    }

    public double getRPM(){
        double distance = Robot.getDistanceFromGoal();
        //Points:
        //92.09: 1620, 3855.6
        //84.7: 1600, 3808
        //100.6: 1680, 3988.4
        //65.55: 1380, 3284.4
        //45.35: 1350, 3213
        //145.88: 2225, 5355
        //156.95: 2280, 5426.5
        //141.15: 2180, 5188.4
        //y=0.0385207x^{2}+1.30815x+1181.56736
        double speed = 0.0385207 * distance * distance + 1.30815 * distance + 1181.56736;
        MyTelem.addData("speed", speed);
        return speed;
    }

    public void setHood(double pos){
        hood.setPosition(pos);
    }
    public double ticksPerSecToRPM(double tps){
        return tps * 60.0 / ShooterConstants.TICKS_PER_REV;
    }
    public void setShooterPIDPower(double targetRPM){
        double topVelocity = Math.abs(shooterMotor.getVelocity());
        double currentRPM = (ticksPerSecToRPM(topVelocity));
        MyTelem.addData("Shooter Current RPM", currentRPM);
        shooterRPMPID.setPID(ShooterConstants.kp, ShooterConstants.ki, ShooterConstants.kd);
        double power = shooterRPMPID.calculate(currentRPM, targetRPM);
        power += (targetRPM > 0) ? (ShooterConstants.kf * (targetRPM / ShooterConstants.MAX_RPM)) : 0.0;
        power = Range.clip(power, 0, 1);
        double currentVoltage = Robot.voltage;shooterMotor.setPower(power * 12.0 / currentVoltage);
    }

    public double CRTicksPerSecToRPM(double tps){
        return tps * 60.0 / ShooterConstants.CR_TICKS_PER_REV;
    }

    public void setCounterRollerPIDPower(double targetRPM){
        double CRVelocity = Math.abs(counterRoller.getVelocity());
        double currentRPM = (CRTicksPerSecToRPM(CRVelocity));
        MyTelem.addData("Counter roller Current RPM", currentRPM);
        counterRPMPID.setPID(ShooterConstants.CRkp, ShooterConstants.CRki, ShooterConstants.CRkd);
        double power = counterRPMPID.calculate(currentRPM, targetRPM);
        power += (targetRPM > 0) ? (ShooterConstants.CRkf * (targetRPM / ShooterConstants.MAX_RPM)) : 0.0;
        power = Range.clip(power, 0, 1);
        double currentVoltage = Robot.voltage;
        MyTelem.addData("Power without voltage modifier", power);
        MyTelem.addData("Power with voltage modifier", power * 12.0 / currentVoltage);
        counterRoller.setPower(power * 12.0 / currentVoltage);
    }

    public boolean shooterAtRPM(){
        return shooterRPMPID.atSetPoint();
    }

    public boolean counterRollerAtRPM() {
        return counterRPMPID.atSetPoint();
    }

    public boolean atRPM() {
        return shooterAtRPM() && counterRollerAtRPM();
    }

    public ShooterState getState(){
        return state;
    }

//    public enum ShooterState{
//        SHOOTING, STOP
//    }

    @Override
    public void periodic() {
        setState(state);
        shooterRPMPID.setPID(ShooterConstants.kp, ShooterConstants.ki, ShooterConstants.kd);
        counterRPMPID.setPID(ShooterConstants.CRkp, ShooterConstants.CRki, ShooterConstants.CRkd);
    }

    public enum ShooterState {
        CLOSE, FAR, STOP, TESTING, CLOSEAUTO, MATH
    }
}
