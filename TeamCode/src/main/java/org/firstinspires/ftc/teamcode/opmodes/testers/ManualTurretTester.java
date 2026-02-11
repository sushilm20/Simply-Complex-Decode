package org.firstinspires.ftc.teamcode.opmodes.testers;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.TurretCommand;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.MyTelem;

@TeleOp(name = "Turret Tester and Tuner")
public class ManualTurretTester extends LinearOpMode {
    public void runOpMode(){
        MyTelem.init(telemetry);
        Robot robot = new Robot(hardwareMap, false);

        GamepadEx gp1 = new GamepadEx(gamepad1);
        CommandScheduler.getInstance().schedule(new TurretCommand(robot, Turret.TurretState.BACK));

        waitForStart();

        while(opModeIsActive()){
            robot.update();
        }
        robot.stop();
    }

}
