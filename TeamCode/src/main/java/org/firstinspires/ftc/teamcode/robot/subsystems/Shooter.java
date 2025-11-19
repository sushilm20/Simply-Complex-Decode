package org.firstinspires.ftc.teamcode.robot.subsystems;

import static org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants.CR_kA;
import static org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants.CR_kS;
import static org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants.CR_kV;
import static org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants.kA;
import static org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants.kS;
import static org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants.kV;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDController;
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





    private double lastShooterTargetRPM = 0;
    private double lastCounterTargetRPM = 0;

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
                double fw = ShooterConstants.tuningTestingRPM;
                double cr = fw * ShooterConstants.counterMultiplier;
                setShooterPIDPower(fw);
                setCounterRollerPIDPower(cr);
                break;
            case CLOSEAUTO:
                setShooterPIDPower(ShooterConstants.closeShootAutoRPM);
                break;
            case MATH:
                double rpm = getRPM();
                MyTelem.addData("Shooter RPM (math target)", rpm);
                setShooterPIDPower(rpm);
                setCounterRollerPIDPower(2.38 * rpm);
                break;
        }
    }

    public double getRPM(){
        double distance = Robot.getDistanceFromGoal();
        // y = 0.0385207x^2 + 1.30815x + 1181.56736
        double speed = 0.0385207 * distance * distance + 1.30815 * distance + 1181.56736;
        MyTelem.addData("Shooter Calc Target RPM", speed);
        return speed;
    }

    public void setHood(double pos){
        hood.setPosition(pos);
    }

    public double ticksPerSecToRPM(double tps){
        return tps * 60.0 / ShooterConstants.TICKS_PER_REV;
    }

    public double CRTicksPerSecToRPM(double tps){
        return tps * 60.0 / ShooterConstants.CR_TICKS_PER_REV;
    }


    public void setShooterPIDPower(double targetRPM){
        double topVelocity = Math.abs(shooterMotor.getVelocity());
        double currentRPM = ticksPerSecToRPM(topVelocity);
        MyTelem.addData("Shooter Current RPM", currentRPM);
        MyTelem.addData("Shooter Target RPM", targetRPM);


        shooterRPMPID.setPID(ShooterConstants.kp, ShooterConstants.ki, ShooterConstants.kd);


        double pidOutput = shooterRPMPID.calculate(currentRPM, targetRPM);


        double accelEstimate = targetRPM - lastShooterTargetRPM;
        double ffOutput = 0.0;

        if (targetRPM > 0) {
            ffOutput = kS
                    + kV * targetRPM
                    + kA * accelEstimate;
        }

        lastShooterTargetRPM = targetRPM;


        double power = pidOutput + ffOutput;


        power = Range.clip(power, 0, 1);

        double currentVoltage = Robot.voltage;
        MyTelem.addData("Shooter Power (no volt comp)", power);
        MyTelem.addData("Shooter Power (volt comp)", power * 12.0 / currentVoltage);

        // voltage compensation so behavior is same at 12.8V vs 12.0V vs 11.5V
        shooterMotor.setPower(power * 12.0 / currentVoltage);
    }


    public void setCounterRollerPIDPower(double targetRPM){
        double CRVelocity = Math.abs(counterRoller.getVelocity());
        double currentRPM = CRTicksPerSecToRPM(CRVelocity);
        MyTelem.addData("Counter roller Current RPM", currentRPM);
        MyTelem.addData("Counter roller Target RPM", targetRPM);

        counterRPMPID.setPID(ShooterConstants.CRkp, ShooterConstants.CRki, ShooterConstants.CRkd);
        double pidOutput = counterRPMPID.calculate(currentRPM, targetRPM);

        double accelEstimate = targetRPM - lastCounterTargetRPM;
        double ffOutput = 0.0;

        if (targetRPM > 0) {
            ffOutput = CR_kS
                    + CR_kV * targetRPM
                    + CR_kA * accelEstimate;
        }

        lastCounterTargetRPM = targetRPM;

        double power = pidOutput + ffOutput;



        power = Range.clip(power, 0, 1);

        double currentVoltage = Robot.voltage;
        MyTelem.addData("CR Power (no volt comp)", power);
        MyTelem.addData("CR Power (volt comp)", power * 12.0 / currentVoltage);

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
