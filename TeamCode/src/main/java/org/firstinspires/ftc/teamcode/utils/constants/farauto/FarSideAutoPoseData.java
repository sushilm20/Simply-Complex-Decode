package org.firstinspires.ftc.teamcode.utils.constants.farauto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;

@Config
public class FarSideAutoPoseData {

    public static final Pose START_POSE =
            new Pose(56.000, 8.000, Math.toRadians(90));

    public static final Pose MID1_CURVE =
            new Pose(53.000, 38.000);

    public static final Pose FIRST_TARGET =
            new Pose(14.000, 36.000);

    public static final Pose SECOND_TARGET =
            new Pose(55.000, 12.000);

    public static final Pose THIRD_TARGET =
            new Pose(10.000, 10.000);

    public static double mirrorX(double x, String color) {
        return color.equals("RED") ? 144 - x : x;
    }
    public static double mirrorHeading(double deg, String color) {
        return color.equals("RED") ? 180 - deg : deg;
    }
    public static Pose mirror(Pose p, String color) {
        if (color.equals("RED")) {
            return new Pose(
                    144 - p.getX(),
                    p.getY(),
                    Math.toRadians(180 - Math.toDegrees(p.getHeading()))
            );
        }
        return p;
    }
}