package org.firstinspires.ftc.teamcode.opmodes.prod;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.botcommands.TransferCancelCommand;
import org.firstinspires.ftc.teamcode.robot.commands.botcommands.TransferCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.BlockerCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.IndexerCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.IntakeCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.ShooterCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.TurretCommand;
import org.firstinspires.ftc.teamcode.robot.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.MyTelem;
import org.firstinspires.ftc.teamcode.utils.constants.BotConstants;

@TeleOp(name = "Teleop Blue", group = "Comp")
public class TeleopBlue extends LinearOpMode {
    @Override
    public void runOpMode() {
        MyTelem.init(telemetry);
        Robot robot = new Robot(hardwareMap, false, "BLUE");
        GamepadEx gp1 = new GamepadEx(gamepad1);
        GamepadEx gp2 = new GamepadEx(gamepad2);

        gp2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new IntakeCommand(robot, Intake.IntakeState.ON));
        gp2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenReleased(new IntakeCommand(robot, Intake.IntakeState.OFF));

        gp2.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new SequentialCommandGroup(
                        new IndexerCommand(robot, Indexer.IndexState.OUT),
                        new WaitCommand(150),
                        new TransferCommand(robot)

                )
        );
        gp2.getGamepadButton(GamepadKeys.Button.B).whenReleased(
                new SequentialCommandGroup(
                        new IndexerCommand(robot, Indexer.IndexState.IN),
                        new TransferCommand(robot),
                        new WaitCommand(100),
                        new TransferCancelCommand(robot)

                )
        );

        gp2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new ParallelCommandGroup(
                        new TurretCommand(robot, Turret.TurretState.MATH),
                        new ShooterCommand(robot, Shooter.ShooterState.SPEEDING_UP),
                        new IntakeCommand(robot, Intake.IntakeState.SOLOFRONT)

                )
        );
        gp2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenReleased(
                new ParallelCommandGroup(
                        new TurretCommand(robot, Turret.TurretState.BACK),
                        new ShooterCommand(robot, Shooter.ShooterState.STOP),
                    new IntakeCommand(robot, Intake.IntakeState.OFF)
                )
        );

        gp2.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new ParallelCommandGroup(
                        new TransferCommand(robot)
                )
        );
        gp2.getGamepadButton(GamepadKeys.Button.Y).whenReleased(
                new ParallelCommandGroup(
                        new TransferCancelCommand(robot)
                )
        );


        gp2.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new ParallelCommandGroup(
                        new BlockerCommand(robot, Blocker.BlockerState.UNBLOCKED),
                        new IntakeCommand(robot, Intake.IntakeState.ON)
                )
        );

        gp2.getGamepadButton(GamepadKeys.Button.A).whenReleased(
                new ParallelCommandGroup(
                        new BlockerCommand(robot, Blocker.BlockerState.BLOCKED),
                        new IntakeCommand(robot, Intake.IntakeState.OFF)
                )
        );

        gp1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand(robot::holding)
        );

        gp1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(()->robot.setResetPose()));

        gp1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenReleased(
                new InstantCommand(robot::stopHolding)
        );


//        gp1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
//                new MoveToCloseShootCommand(robot)
//        );
//
//        gp1.getGamepadButton(GamepadKeys.Button.A).whenReleased(
//                new InstantCommand(robot::stopHolding)
//        );
        CommandScheduler.getInstance().schedule(new BlockerCommand(robot, Blocker.BlockerState.BLOCKED));
        waitForStart();

        if(isStarted()){
            robot.follower.startTeleopDrive();
        }

        while (opModeIsActive()) {
            robot.follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x * BotConstants.turnSpeed, true);
            robot.update();
        }

        robot.stop();
    }
}
