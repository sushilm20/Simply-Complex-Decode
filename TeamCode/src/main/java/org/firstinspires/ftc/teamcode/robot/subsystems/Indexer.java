package org.firstinspires.ftc.teamcode.robot.subsystems;

import static org.firstinspires.ftc.teamcode.utils.constants.IndexerConstants.inPosition;
import static org.firstinspires.ftc.teamcode.utils.constants.IndexerConstants.outPosition;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.Servo;

public class Indexer implements Subsystem {
    public Servo indexServo;
    public IndexState state;
    public Indexer(Servo indexServo) {
        this.indexServo = indexServo;
        setState(IndexState.IN);
    }

    public void setState(IndexState state) {
        this.state = state;
        switch (state) {
//            case SORT:

            case IN:
                indexServo.setPosition(inPosition);
                break;
            case OUT:
                indexServo.setPosition(outPosition);
                break;
        }
    }


    public IndexState getState() {return state;}

    public enum IndexState{
        IN, OUT,
        SORT
    }
}
