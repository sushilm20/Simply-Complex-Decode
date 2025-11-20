package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.utils.constants.IntakeConstants;

public class Intake implements Subsystem {
    DcMotorEx intakeMotor2;
    public IntakeState state;
    public Intake(DcMotorEx intakeMotor2){
        this.intakeMotor2 = intakeMotor2;
        intakeMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setState(IntakeState state){
        this.state = state;
        switch (state){
            case ON:
                intakeMotor2.setPower(IntakeConstants.forwardPower);
                break;
            case OFF:
                intakeMotor2.setPower(IntakeConstants.offPower);
                break;
            case REV:
                intakeMotor2.setPower(IntakeConstants.backwardPower);
                break;
            default:
                intakeMotor2.setPower(0);
                break;
        }
    }

    public IntakeState getState(){
        return state;
    }
    public enum IntakeState{
        ON, OFF, REV;
    }

}
