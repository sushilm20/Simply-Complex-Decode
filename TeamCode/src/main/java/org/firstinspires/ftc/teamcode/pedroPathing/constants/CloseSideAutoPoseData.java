package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.Pose;

public class CloseSideAutoPoseData {
    // Blue-side coordinate definitions (field center at 72,72, field is 144x144)
    public static final Pose START_POSE     = new Pose(19.250, 123.000, Math.toRadians(143));
    public static final Pose SHOOTING_POSE  = new Pose(AutoConstants.shootingX, AutoConstants.shootingY);
    public static final Pose MID1_CURVE     = new Pose(56.86, 78.80);     // path2 control point
    public static final Pose FIRST_INTAKE   = new Pose(AutoConstants.firstIntakeX, 84.000);
    public static final Pose MID2_CURVE     = new Pose(66, 52);           // path4 control point
    public static final Pose SECOND_INTAKE  = new Pose(AutoConstants.secondIntakeX, 60.000);
    public static final Pose MID3_CURVE     = new Pose(87, 31);           // path6 control point
    public static final Pose FINAL_INTAKE   = new Pose(AutoConstants.secondIntakeX, 36.000);
    public static final Pose TRAILER_FINISH = new Pose(13.000, 36.000);   // for last blue path
    public static final double START_HEADING  = 143;
    public static final double SHOOTING_HEADING = AutoConstants.shootingAngle;
    // Add any more as needed...

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
