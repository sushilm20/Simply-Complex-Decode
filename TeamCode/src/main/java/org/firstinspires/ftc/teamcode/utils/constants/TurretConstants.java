package org.firstinspires.ftc.teamcode.utils.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class TurretConstants {
    // HORS motor-based turret PID constants
    public static int TURRET_MIN_POS = -1000;
    public static int TURRET_MAX_POS = 1000;
    public static double TICKS_PER_RADIAN_SCALE = 0.87;

    public static double TURRET_KP = 1.0;
    public static double TURRET_KI = 0.0;
    public static double TURRET_KD = 0.235;
    public static double TURRET_MAX_POWER = 1.0;

    public static double FF_GAIN = 5.0;
    public static double POWER_SMOOTH_ALPHA = 0.9;
    public static double DERIV_FILTER_ALPHA = 1.0;

    public static int SMALL_DEADBAND_TICKS = 3;
    public static double INTEGRAL_CLAMP = 50.0;

    public static double RIGHTWARD_ENCODER_DAMP = 0.9;
    public static int RIGHTWARD_DAMP_ERROR_WINDOW = 50;

    // Homing sweep
    public static int HOMING_AMPLITUDE_TICKS = 300;
    public static double HOMING_POWER = 0.5;
    public static int HOMING_TARGET_DEADBAND = 12;
    public static long HOMING_TIMEOUT_MS = 3000;

    // Manual turret power
    public static double MANUAL_TURRET_POWER = 0.35;

    // Legacy fields kept for compatibility with existing auto path callers
    public static double turretForwardPosition = 0.5;
    public static double SLOPE = -0.00244444;
    public static double OFFSET = 0.5;
    public static double MAX_STEP_PER_LOOP = 0.02;
    public static double tolerance = 0.001;
    public static double P = 0.08, I = 0.0, D = 0, F = 0.0;
    public static double closeTolerance = 6;
}
