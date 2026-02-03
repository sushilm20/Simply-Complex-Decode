package org.firstinspires.ftc.teamcode.utils.constants;

import static org.firstinspires.ftc.teamcode.utils.constants.auto.AutoConstants.startX;
import static org.firstinspires.ftc.teamcode.utils.constants.auto.AutoConstants.startY;

import com.acmerobotics.dashboard.config.Config;
@Config
public class BotConstants {
    public static double goalX = 144;
    public static double goalY = 144;
    public static double goalDY = 24; //correct later
    public static double cameraYaw = 12;
    public static double turnSpeed = 0.7;
    public enum BotState{
        MATH, MANUAL, TESTING
    }
}
