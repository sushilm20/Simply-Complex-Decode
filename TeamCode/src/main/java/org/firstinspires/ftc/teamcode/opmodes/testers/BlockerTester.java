package org.firstinspires.ftc.teamcode.opmodes.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.MyTelem;

@Config
@TeleOp(name = "Blocker Tester")
public class BlockerTester extends LinearOpMode {
    public static String hardwareMapName = "BlockerServo";
    public static double servoPos = 0.0;

    public void runOpMode() throws InterruptedException{
        Servo BlockerServo = hardwareMap.get(Servo.class, hardwareMapName);
        double currentServoPos = servoPos;
        waitForStart();
        while (opModeIsActive()){
            if (servoPos != currentServoPos){
                BlockerServo.setPosition(servoPos);
            }
            currentServoPos = servoPos;
        }
    }

}
