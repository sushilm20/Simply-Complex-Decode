package org.firstinspires.ftc.teamcode.opmodes.testers;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.utils.MyTelem;
import org.firstinspires.ftc.teamcode.utils.constants.HoodConstants;
import org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants;

@Config
@TeleOp(name = "Hood Tester")
public class HoodAngleTest extends LinearOpMode{
    @Override
    public void runOpMode(){
        MyTelem.init(telemetry);
        Robot robot = new Robot(hardwareMap, false);

        waitForStart();

        while (opModeIsActive()) {
            robot.shooter.state = Shooter.ShooterState.TESTING;
            MyTelem.addLine("--- SHOOTER ---");
            MyTelem.addData("At RPM", robot.shooter.shooterAtRPM());
            MyTelem.addLine();
            robot.update();
        }
    }
}
