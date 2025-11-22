package org.firstinspires.ftc.teamcode.utils.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConstants {
    public static double closeShootRPM = 1950;// 1800
    public static double farShootRPM = 3000; //2.16k
    public static double shootingMultiplier = 3;
    public static double kf = 0, kp = 0.05, ki = 0, kd = 0.0001;
    public static double tuningTestingRPM = 0;
    public static double tuningTestingCounterRollerRPM = 0;
    public static int TICKS_PER_REV = 28;
    public static int MAX_RPM = 6000;
    public static double CRkf = 0.3, CRkp = 2.0, CRki = 0, CRkd = 0;
    public static int CR_TICKS_PER_REV = 28;



}
