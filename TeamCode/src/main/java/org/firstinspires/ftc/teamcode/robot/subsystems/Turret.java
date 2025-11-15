package org.firstinspires.ftc.teamcode.robot.subsystems;

import static com.sun.tools.javac.api.DiagnosticFormatter.PositionKind.OFFSET;

import static org.firstinspires.ftc.teamcode.utils.constants.TurretConstants.SLOPE;

import com.arcrobotics.ftclib.command.Subsystem;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.MyTelem;
import org.firstinspires.ftc.teamcode.utils.constants.TurretConstants;

public class Turret implements Subsystem {

    public Servo turretLeftServo, turretRightServo;
    public TurretState state;

    // === Exact mapping from calibration ===
    // Forward = 0.5  at   0°
    // 180°    = 0.42
    // slope = 0.0032222 (servoPos/degree)

    public Turret(Servo turretLeftServo, Servo turretRightServo) {
        this.turretLeftServo = turretLeftServo;
        this.turretRightServo = turretRightServo;

        state = TurretState.FRONT;
    }

    @Override
    public void periodic() {
        setState(state);
    }

    public void setState(TurretState state) {
        this.state = state;

        switch (state) {
            case FRONT:
                setServoPos(TurretConstants.turretForwardPosition);
                break;

            case MATH:
                pointToGoal();
                break;
        }
    }
    private void setServoPos(double pos) {
        pos = Range.clip(pos, 0, 1);
        turretLeftServo.setPosition(pos);
        turretRightServo.setPosition(pos);
    }
    private void pointToGoal() {
        Pose cur  = Robot.currentPose;
        Pose goal = Robot.getGoalPose();
        double fieldAngle = Math.atan2(
                goal.getY() - cur.getY(),
                goal.getX() - cur.getX()
        );
        double relAngle = fieldAngle - cur.getHeading();
        while (relAngle > Math.PI)  relAngle -= 2 * Math.PI;
        while (relAngle < -Math.PI) relAngle += 2 * Math.PI;
        double angleDeg = Math.toDegrees(relAngle);
        double servoPos = TurretConstants.OFFSET + TurretConstants.SLOPE * angleDeg;
        if (servoPos > 1.0) servoPos = 1.0;
        if (servoPos < 0.0) servoPos = 0.0;
        setServoPos(servoPos);
        MyTelem.addData("Turret Servo Position", servoPos);
        MyTelem.addData("Turret Angle", angleDeg);
    }

    public TurretState getState() {
        return state;
    }
    public enum TurretState {
        FRONT, MATH, BACK
    }
}
