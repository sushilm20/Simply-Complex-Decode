package org.firstinspires.ftc.teamcode.robot.subsystems;

import static org.firstinspires.ftc.teamcode.utils.constants.IndexerConstants.inPosition;
import static org.firstinspires.ftc.teamcode.utils.constants.IndexerConstants.outPosition;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.constants.IndexerConstants;

/**
 * HORS claw subsystem (repurposed from Indexer).
 * IN = claw open (0.62), OUT = claw closed (0.3).
 * Timed close: closes for CLAW_CLOSE_MS then reopens automatically.
 */
public class Indexer implements Subsystem {
    public Servo clawServo;
    public IndexState state;
    private boolean timedCloseActive = false;
    private long timedCloseStartMs = 0L;

    public Indexer(Servo clawServo) {
        this.clawServo = clawServo;
        setState(IndexState.IN);
    }

    public void setState(IndexState state) {
        this.state = state;
        switch (state) {
            case IN:
                clawServo.setPosition(inPosition); // open
                timedCloseActive = false;
                break;
            case OUT:
                clawServo.setPosition(outPosition); // closed
                timedCloseActive = true;
                timedCloseStartMs = System.currentTimeMillis();
                break;
        }
    }

    @Override
    public void periodic() {
        // Auto-reopen claw after timed close
        if (timedCloseActive
                && System.currentTimeMillis() - timedCloseStartMs >= IndexerConstants.CLAW_CLOSE_MS) {
            clawServo.setPosition(inPosition);
            timedCloseActive = false;
            state = IndexState.IN;
        }
    }

    public IndexState getState() {return state;}

    public enum IndexState{
        IN, OUT,
        SORT
    }
}
