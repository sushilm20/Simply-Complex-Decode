package org.firstinspires.ftc.teamcode.utils.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConstants {
    public static double currentVelocity = 0.0;
    public static double startingVelocity = 0.0;
    public static double speedingVelocity = 4000;
    public static double closeShootRPM = 4100;// 1800
    public static double farShootRPM = 4100; //2.16k
    public static double tuningTestingRPM = 4100;
    public static double kf = 0.5, kp = 0.4, ki = 0, kd = 0.0000001;
//    public static double CRkf = 0.3, CRkp = 0.004, CRki = 0, CRkd = 0.00001;
//    public static int CR_TICKS_PER_REV = 28;
    public static int RPM_OFFSET = 50;
    public static int TICKS_PER_REV = 28;
    public static int MAX_RPM = 6000;





}
