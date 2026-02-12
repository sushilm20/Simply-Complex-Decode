package org.firstinspires.ftc.teamcode.robot.subsystems;

import static org.firstinspires.ftc.teamcode.utils.constants.TurretConstants.MAX_STEP_PER_LOOP;
import static org.firstinspires.ftc.teamcode.utils.constants.TurretConstants.P;
import static org.firstinspires.ftc.teamcode.utils.constants.TurretConstants.SLOPE;
import static org.firstinspires.ftc.teamcode.utils.constants.TurretConstants.closeTolerance;

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
    private double turretCommandPos = TurretConstants.turretForwardPosition;

    // === Exact mapping from calibration ===
    // Forward = 0.5  at   0°
    // 180°    = 0.06 0.44
    // slope = 0.002444444 (servoPos/degree)
    // new slope = -0.002444444

    public Turret(Servo turretLeftServo, Servo turretRightServo) {
        this.turretLeftServo = turretLeftServo;
        this.turretRightServo = turretRightServo;
    }

    public void setState(TurretState state) {
        if (state == null) return;
        this.state = state;
        switch (state) {
            case BACK:
                setServoPos(TurretConstants.turretForwardPosition);
                break;
            case MATH:
                pointToGoalMath();
                break;
//            case MATH_CAMERA:
//                LimelightCamera.TagTarget tag = Robot.getTargetTag();
//                if (tag == null || !tag.hasTarget) {
//                    pointToGoalPinPoint(Robot.getEffectiveCoordinates());
//                } else{
//                    if (Math.abs(tag.tX) > closeTolerance && Robot.auto) {
//                        pointToGoalPinPoint(Robot.getEffectiveCoordinates());
//                        return;
//                    }
//                    pointToGoalCamera(tag);
//                }
//                break;
        }
    }
    private void setServoPos(double pos) {
        pos = Range.clip(pos, 0, 1);
        turretCommandPos = pos;
        turretLeftServo.setPosition(pos);
        turretRightServo.setPosition(pos);
    }
    private void pointToGoalMath() {
//        double angleDeg = -1 * Robot.getTurretAngle(); //its like opposite idk why but just cuz
        double angleDeg = Robot.getTurretAngle() - 180.0; //its like opposite idk why but just cuz

        double servoPos = TurretConstants.OFFSET + TurretConstants.SLOPE * angleDeg;

        if (servoPos > 1.0) servoPos = 1.0;
        if (servoPos < 0.0) servoPos = 0.0;
        setServoPos(servoPos);
    }

    private void pointToGoalPinPoint(Pose cur) {
        Pose goal = Robot.getGoalPose();
        double fieldAngle = Math.atan2(
                goal.getY() - cur.getY(),
                goal.getX() - cur.getX()
        );
        double relAngle = fieldAngle - cur.getHeading() - Math.PI;
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

    private void pointToGoalCamera(LimelightCamera.TagTarget tag) {
        if (tag == null || !tag.hasTarget) return;
        double tX = tag.tX;
        if (Math.abs(tX) < 0.5){
            return;
        }
        double deltaPos = tX * SLOPE * P;
        double targetAngleDeg = turretCommandPos - deltaPos;

        double step = Range.clip(targetAngleDeg - turretCommandPos, -MAX_STEP_PER_LOOP, MAX_STEP_PER_LOOP);
        setServoPos(turretCommandPos + step);


        MyTelem.addData("Math Camera", true);
    }


    public TurretState getState() {
        return state;
    }
    @Override
    public void periodic(){
        setState(state);
    }

    public enum TurretState {
        BACK, MATH
    }
}
