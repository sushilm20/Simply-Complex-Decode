package org.firstinspires.ftc.teamcode.utils.constants.auto;

import static org.firstinspires.ftc.teamcode.utils.constants.auto.AutoConstants.leverIntakeX;
import static org.firstinspires.ftc.teamcode.utils.constants.auto.AutoConstants.leverIntakeY;
import static org.firstinspires.ftc.teamcode.utils.constants.auto.AutoConstants.leverX;
import static org.firstinspires.ftc.teamcode.utils.constants.auto.AutoConstants.leverY;
import static org.firstinspires.ftc.teamcode.utils.constants.auto.AutoConstants.leverYNew;
import static org.firstinspires.ftc.teamcode.utils.constants.auto.AutoConstants.startHeading;
import static org.firstinspires.ftc.teamcode.utils.constants.auto.AutoConstants.startX;
import static org.firstinspires.ftc.teamcode.utils.constants.auto.AutoConstants.startY;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;

@Config
public class CloseSideAutoPoseData {
    public static final Pose LEVER_SECOND     = new Pose(leverX, leverYNew);

    public static final Pose START_POSE     = new Pose(startX, startY, Math.toRadians(startHeading));
    public static final Pose SHOOTING_POSE  = new Pose(AutoConstants.shootingX, AutoConstants.shootingY);
    public static final Pose MID1_CURVE     = new Pose(52.3, 85);     // path2 control point
    public static final Pose FIRST_INTAKE   = new Pose(AutoConstants.firstIntakeX, 85.500);
    public static final Pose MID2_CURVE     = new Pose(66, 52);           // path4 control point
    public static final Pose SECOND_INTAKE  = new Pose(AutoConstants.secondIntakeX, AutoConstants.secondIntakeY);
    public static final Pose LEVER = new Pose(AutoConstants.leverX, AutoConstants.leverY);
    public static final Pose LEVER_CONTROL = new Pose(AutoConstants.leverPoseX, AutoConstants.leverPoseY);
    public static final Pose MID3_CURVE    = new Pose(79, 28);           // path6 control point
    public static final Pose FINAL_INTAKE   = new Pose(AutoConstants.secondIntakeX, 37.5);
    public static final Pose FINAL_SHOOT   = new Pose(63, 102);
    public static final Pose LEVER_INTAKE = new Pose(leverIntakeX, leverIntakeY);
    public static final double SHOOTING_HEADING = AutoConstants.shootingAngle;
    public static double mirrorX(double x, String color) {
        return color.equals("RED") ? 144 - x : x;
    }
    public static double mirrorHeading(double deg, String color) {
        return color.equals("RED") ? 180 - deg : deg;
    }
    public static Pose mirror(Pose p, String color) {
        return color.equals("RED") ? new Pose(144 - p.getX(), p.getY(), Math.toRadians(180 - Math.toDegrees(p.getHeading()))) : p;
    }
}
