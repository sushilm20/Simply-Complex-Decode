package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.utils.constants.KickerConstants;

public class Kicker implements Subsystem {
    public CRServo rightServo, leftServo;
    public KickerState state;

    public Kicker(CRServo rightServo, CRServo leftServo) {
        this.leftServo = leftServo;
        this.rightServo = rightServo;
        state = KickerState.OFF;
        rightServo.setDirection(CRServo.Direction.REVERSE);
        leftServo.setDirection(CRServo.Direction.REVERSE);
    }

    public void setState(KickerState state) {
        this.state = state;
        switch (state) {
            case ON:
                leftServo.setPower(KickerConstants.onPower);
                rightServo.setPower(KickerConstants.onPower); // reversed
                break;
            case OFF:
                leftServo.setPower(KickerConstants.offPower);
                rightServo.setPower(KickerConstants.offPower);
                break;
            default:
                leftServo.setPower(KickerConstants.offPower);
                rightServo.setPower(KickerConstants.offPower);
                break;
        }
    }

    public KickerState getState() {
        return state;
    }

    public enum KickerState {
        ON, OFF
    }
}
