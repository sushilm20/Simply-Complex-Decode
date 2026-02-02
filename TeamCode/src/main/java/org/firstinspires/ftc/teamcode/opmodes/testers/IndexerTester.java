package org.firstinspires.ftc.teamcode.opmodes.testers;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.IndexerCommand;
import org.firstinspires.ftc.teamcode.robot.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.utils.MyTelem;

@Config
@TeleOp(name = "Indexer Tester", group = " ")
public class IndexerTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MyTelem.init(telemetry);
        Robot robot = new Robot(hardwareMap, false);
        GamepadEx gp1 = new GamepadEx(gamepad1);
        gp1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new IndexerCommand(robot, Indexer.IndexState.OUT)

        );
        gp1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenReleased(
                new IndexerCommand(robot, Indexer.IndexState.IN)

        );

        waitForStart();
        while (opModeIsActive()){
            robot.update();
        }
        robot.stop();
    }
}
