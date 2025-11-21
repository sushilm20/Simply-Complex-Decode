package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.utils.constants.IntakeConstants;

public class Intake implements Subsystem {
    DcMotorEx intakeMotor, intakeMotor2;
    public IntakeState state;
    public Intake(DcMotorEx intakeMotor, DcMotorEx intakeMotor2){
        this.intakeMotor2 = intakeMotor2;
        this.intakeMotor = intakeMotor;
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setState(IntakeState state){
        this.state = state;
        switch (state){
            case ON:
                intakeMotor2.setPower(IntakeConstants.forwardPower);
                intakeMotor.setPower(IntakeConstants.forwardPower);
                break;
            case SOLOFRONT:
                intakeMotor.setPower(IntakeConstants.forwardPower);
                intakeMotor2.setPower(IntakeConstants.offPower);
                break;
            case OFF:
                intakeMotor2.setPower(IntakeConstants.offPower);
                intakeMotor.setPower(IntakeConstants.offPower);
                break;
            case REV:
                intakeMotor.setPower(IntakeConstants.backwardPower);
                intakeMotor2.setPower(IntakeConstants.backwardPower);
                break;
        }
    }

    public IntakeState getState(){
        return state;
    }
    public enum IntakeState{
        ON, OFF, REV, SOLOFRONT;
    }

}
