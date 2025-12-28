package org.firstinspires.ftc.teamcode.utils.constants;

import static org.firstinspires.ftc.teamcode.utils.constants.AutoConstants.startX;
import static org.firstinspires.ftc.teamcode.utils.constants.AutoConstants.startY;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;
@Config
public class CloseSideAutoPoseData {
    public static final Pose START_POSE     = new Pose(startX, startY, Math.toRadians(143.7));
    public static final Pose SHOOTING_POSE  = new Pose(AutoConstants.shootingX, AutoConstants.shootingY);
    public static final Pose MID1_CURVE     = new Pose(52.3, 85);     // path2 control point
    public static final Pose FIRST_INTAKE   = new Pose(AutoConstants.firstIntakeX, 86.000);
    public static final Pose MID2_CURVE     = new Pose(66, 52);           // path4 control point
    public static final Pose SECOND_INTAKE  = new Pose(AutoConstants.secondIntakeX, 58.5);
    public static final Pose LEVER = new Pose(AutoConstants.leverX, AutoConstants.leverY);
    public static final Pose LEVER_CONTROL = new Pose(AutoConstants.leverPoseX, AutoConstants.leverPoseY);
    public static final Pose MID3_CURVE     = new Pose(87, 31);           // path6 control point
    public static final Pose FINAL_INTAKE   = new Pose(AutoConstants.secondIntakeX, 34);
    public static final Pose FINAL_SHOOT   = new Pose(69, 95.5);

    public static final double START_HEADING  = AutoConstants.startHeading;
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
