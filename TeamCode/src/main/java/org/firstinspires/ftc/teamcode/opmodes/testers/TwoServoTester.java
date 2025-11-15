package org.firstinspires.ftc.teamcode.opmodes.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Two Servo Tester")
public class TwoServoTester extends LinearOpMode {
    public static String regServo = "";
    public static String revServo = "";
    public static double position = 0.1;
    public void runOpMode(){

        Servo a = hardwareMap.get(Servo.class, regServo);
        Servo b = hardwareMap.get(Servo.class, revServo);
        waitForStart();
        while(opModeIsActive()){
            a.setPosition(position);
            b.setPosition(position);
        }
    }
}
