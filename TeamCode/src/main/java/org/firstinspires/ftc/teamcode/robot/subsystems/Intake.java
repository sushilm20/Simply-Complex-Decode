package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.utils.MyTelem;
import org.firstinspires.ftc.teamcode.utils.constants.IntakeConstants;

public class Intake implements Subsystem {
    DcMotor intakeMotor;
    public IntakeState state;

    public Intake(DcMotor intakeMotor){
        this.intakeMotor = intakeMotor;
    }

    public void setState(IntakeState state){
        if(state == null){
            return;
        }
        this.state = state;
        switch (state){
            case ON:
                intakeMotor.setPower(IntakeConstants.forwardPower);
                break;
            case OFF:
                intakeMotor.setPower(IntakeConstants.offPower);
                break;
            case REVERSE:
                intakeMotor.setPower(IntakeConstants.backwardPower);
                break;
            case SOLOFRONT:
                // Same as ON for single-motor HORS intake
                intakeMotor.setPower(IntakeConstants.forwardPower);
                break;
        }
    }
    @Override
    public void periodic(){
        setState(state);
    }

    public IntakeState getState(){
        return state;
    }
    public enum IntakeState{
        ON, OFF, SOLOFRONT, REVERSE
    }

}
