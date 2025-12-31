package org.firstinspires.ftc.teamcode.utils.constants;

import com.acmerobotics.dashboard.config.Config;
@Config
public class BotConstants {
    public static double goalX = 122.69;
    public static double goalY = 123.85;
    public static double goalDY = 20; //correct later
    public static double cameraYaw = 13;
    public static double turnSpeed = 0.7;
    public enum BotState{
        MATH, MANUAL, TESTING
    }
}
