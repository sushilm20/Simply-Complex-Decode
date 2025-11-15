package org.firstinspires.ftc.teamcode.opmodes.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.utils.MyTelem;
import org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants;

@Config
@TeleOp(name = "Shooter Tuner")
public class ShooterTuner extends LinearOpMode {
    public void runOpMode(){
        MyTelem.init(telemetry);
        Robot robot = new Robot(hardwareMap, false);
        robot.shooter.state = Shooter.ShooterState.TESTING;
        waitForStart();
        while (opModeIsActive()) {
            MyTelem.addLine("--- SHOOTER ---");
            MyTelem.addData("Target RPM", ShooterConstants.tuningTestingRPM);
            MyTelem.addData("At RPM", robot.shooter.shooterAtRPM());
            MyTelem.addLine();
            MyTelem.addLine("--- COUNTER ROLLER ---");
            MyTelem.addData("Target RPM", ShooterConstants.tuningTestingCounterRollerRPM);
            MyTelem.addData("At RPM", robot.shooter.counterRollerAtRPM());
            robot.update();
        }
        robot.stop();
    }
}
