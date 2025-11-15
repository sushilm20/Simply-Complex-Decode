package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.constants.BlockerConstants;
import org.firstinspires.ftc.teamcode.utils.constants.KickerConstants;

public class Blocker {
        public Servo servo;
        public org.firstinspires.ftc.teamcode.robot.subsystems.Blocker.BlockerState state;
        public Blocker(Servo servo) {
            this.servo = servo;
            state = org.firstinspires.ftc.teamcode.robot.subsystems.Blocker.BlockerState.BLOCKED;
        }

        public void setState(org.firstinspires.ftc.teamcode.robot.subsystems.Blocker.BlockerState state) {
            this.state = state;
            switch (state) {
                case BLOCKED:
                    servo.setPosition(BlockerConstants.Blocked);
                    break;
                case UNBLOCKED:
                default:
                    servo.setPosition(BlockerConstants.Unblocked);
                    break;
            }
        }

        public org.firstinspires.ftc.teamcode.robot.subsystems.Blocker.BlockerState getState() {
            return state;
        }

        public enum BlockerState {
            BLOCKED, UNBLOCKED
        }
    }

}
