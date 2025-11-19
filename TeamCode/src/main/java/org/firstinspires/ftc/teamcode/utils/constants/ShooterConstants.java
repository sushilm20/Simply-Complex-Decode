package org.firstinspires.ftc.teamcode.utils.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConstants {
    public static double closeShootRPM = 1700;// 1800
    public static double closeShootCounterRollerRPM = 4000;
    public static double closeShootAutoRPM = 1700;
    public static double farShootRPM = 2000; //2.16k
    public static double farShootCounterRollerRPM = 5000;
    public static double kf = 0.8, kp = 20, ki = 0, kd = 0;
    public static double tuningTestingRPM = 0;

    public static double kS = 0.02;
    public static double kV = 0.00027;
    public static double kA = 0.0;

    public static double CR_kS = 0.02;
    public static double CR_kV = 0.000225;
    public static double CR_kA = 0.0;
    //public static double tuningTestingCounterRollerRPM = 0;
    public static double counterMultiplier = 2.8;
    public static int TICKS_PER_REV = 28;
    public static int MAX_RPM = 6000;
    public static double CRkf = 0.3, CRkp = 10, CRki = 0, CRkd = 0;
    public static int CR_TICKS_PER_REV = 28;



}
