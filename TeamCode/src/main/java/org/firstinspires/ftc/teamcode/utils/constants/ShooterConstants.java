package org.firstinspires.ftc.teamcode.utils.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConstants {
    public static double currentVelocity = 0.0;
    public static double startingVelocity = 0.0;
    public static double speedingVelocity = 2600;
    public static boolean karthikstfu = false;
    public static double closeShootRPM = 2600;
    public static double farShootRPM = 3390;
    public static double tuningTestingRPM = 2600;

    // HORS PIDF — CLOSE mode (target RPM < RPM_SWITCH_THRESHOLD)
    public static double CLOSE_kP = 0.00146;
    public static double CLOSE_kI = 0.0027;
    public static double CLOSE_kD = 0.00002;
    public static double CLOSE_kF = 1.72;
    public static double CLOSE_integralLimit = 50;
    public static double CLOSE_derivativeAlpha = 0.9;
    public static double CLOSE_rpmFilterAlpha = 0.72;
    public static double CLOSE_powerSmoothingAlpha = 0.5;
    public static double CLOSE_ffReferenceVoltage = 13.0;
    public static double CLOSE_ffReferenceMaxTicksPerSec = 4930;
    public static double CLOSE_rpmTolerance = 50.0;

    // HORS PIDF — FAR mode (target RPM >= RPM_SWITCH_THRESHOLD)
    public static double FAR_kP = 0.00158;
    public static double FAR_kI = 0.0040;
    public static double FAR_kD = 0.00001;
    public static double FAR_kF = 1.92;
    public static double FAR_integralLimit = 50;
    public static double FAR_derivativeAlpha = 0.72;
    public static double FAR_rpmFilterAlpha = 0.9;
    public static double FAR_powerSmoothingAlpha = 0.5;
    public static double FAR_ffReferenceVoltage = 13.0;
    public static double FAR_ffReferenceMaxTicksPerSec = 4900;
    public static double FAR_rpmTolerance = 50.0;

    public static double RPM_SWITCH_THRESHOLD = 3000.0;

    // Legacy PID fields kept for compatibility with existing callers
    public static double kf = 0.000175, kp = 0.00146, ki = 0.0027, kd = 0.00002;
    public static int RPM_OFFSET = 50;
    public static int TICKS_PER_REV = 28;
    public static int MAX_RPM = 6000;
    public static double hoodServoPosition = 0;
    public static double closeHoodAngle = 0.32;
    public static double openAngle = 0.17;
    public static double tuningTestingHoodPosition = 0.32;
    public static double rpmTolerance = 45.0;
}
