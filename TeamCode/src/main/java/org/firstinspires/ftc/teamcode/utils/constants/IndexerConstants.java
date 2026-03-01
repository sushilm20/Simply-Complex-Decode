package org.firstinspires.ftc.teamcode.utils.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class IndexerConstants {
    // HORS claw servo positions (repurposed from Indexer)
    public static double inPosition = 0.62;   // claw open
    public static double outPosition = 0.3;   // claw closed
    public static long CLAW_CLOSE_MS = 400;    // timed close duration

    public static double greenRatio = 1;
    public static double greenThreshold = 150;

    public static double purpleRedThreshold = 100;
    public static double purpleBlueThreshhold = 100;
    public static double purpleRatio = 1;
}
