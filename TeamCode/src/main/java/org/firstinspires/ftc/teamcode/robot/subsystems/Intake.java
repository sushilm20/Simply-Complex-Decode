package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.utils.MyTelem;
import org.firstinspires.ftc.teamcode.utils.constants.IntakeConstants;

public class Intake implements Subsystem {
    DcMotorEx intakeMotor, intakeMotor2;
    public IntakeState state;
    public Intake(DcMotorEx intakeMotor, DcMotorEx intakeMotor2){
        this.intakeMotor2 = intakeMotor2;
        this.intakeMotor = intakeMotor;
        intakeMotor2.setCurrentAlert(4.5, CurrentUnit.AMPS);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setState(IntakeState state){
        if(state == null){
            return;
        }
        this.state = state;
        switch (state){
            case ON:
                intakeMotor.setPower(IntakeConstants.forwardPower);
//                if (intakeMotor2.isOverCurrent()) {
//                    intakeMotor2.setPower(IntakeConstants.offPower);
//                    MyTelem.addData("someone", "is not stupid af");
//                }
//                else {
                    intakeMotor2.setPower(IntakeConstants.forwardPower);
//                }
                break;
            case OFF:
                intakeMotor.setPower(IntakeConstants.offPower);
                intakeMotor2.setPower(IntakeConstants.offPower);
                break;
            case SOLOFRONT:
                intakeMotor.setPower(IntakeConstants.forwardPower);
                intakeMotor2.setPower(IntakeConstants.offPower);
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
        ON, OFF, SOLOFRONT
    }

}
