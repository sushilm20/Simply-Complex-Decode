package org.firstinspires.ftc.teamcode.utils.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConstants {
    public static double currentVelocity = 0.0;
    public static double startingVelocity = 0.0;
    public static double speedingVelocity = 4100;
    public static boolean karthikstfu = false;
    public static double closeShootRPM = 4100;// 1800
    public static double tuningTestingRPM = 4100;
    public static double kf = 0, kp = 100000, ki = 0, kd = 0;
    public static int RPM_OFFSET = 50;
    public static int TICKS_PER_REV = 28;
    public static int MAX_RPM = 6000;
    public static double hoodServoPosition = 0;
    public static double closeHoodAngle = 0.32;
    public static double openAngle = 0.17;
    public static double tuningTestingHoodPosition = 0.32;
}
