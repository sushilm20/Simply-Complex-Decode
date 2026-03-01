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
import org.firstinspires.ftc.teamcode.robot.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.MyTelem;
import org.firstinspires.ftc.teamcode.utils.constants.BotConstants;
import org.firstinspires.ftc.teamcode.utils.constants.TurretConstants;

@TeleOp(name = "Teleop Red", group = "Comp")
public class TeleopRed extends LinearOpMode {
    @Override
    public void runOpMode() {
        MyTelem.init(telemetry);
        Robot robot = new Robot(hardwareMap, false, "RED");
        GamepadEx gp1 = new GamepadEx(gamepad1);
        GamepadEx gp2 = new GamepadEx(gamepad2);

        // ═══ GP2: Intake on/off (RIGHT_BUMPER) ═══
        gp2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new IntakeCommand(robot, Intake.IntakeState.ON));
        gp2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenReleased(new IntakeCommand(robot, Intake.IntakeState.OFF));

        // ═══ GP2 B: Toggle gate ═══
        gp2.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new BlockerCommand(robot, Blocker.BlockerState.UNBLOCKED)
        );
        gp2.getGamepadButton(GamepadKeys.Button.B).whenReleased(
                new BlockerCommand(robot, Blocker.BlockerState.BLOCKED)
        );

        // ═══ GP2 X: Manual claw trigger ═══
        gp2.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new IndexerCommand(robot, Indexer.IndexState.OUT)
        );

        // ═══ GP2 Y: Start intake auto-sequence (transfer) ═══
        gp2.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new TransferCommand(robot)
        );
        gp2.getGamepadButton(GamepadKeys.Button.Y).whenReleased(
                new TransferCancelCommand(robot)
        );

        // ═══ GP2 DPAD_UP: Aim mode (turret + shooter spin-up) ═══
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

        // ═══ GP2 DPAD_DOWN: Toggle shooter on/off ═══
        gp2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new ShooterCommand(robot, Shooter.ShooterState.SPEEDING_UP)
        );

        // ═══ GP2 A: Unblock gate + intake ═══
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

        // ═══ GP1: Drive hold/release ═══
        gp1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand(robot::holding)
        );
        gp1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenReleased(
                new InstantCommand(robot::stopHolding)
        );

        // ═══ GP1 DPAD_DOWN: Reset pose ═══
        gp1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(()->robot.setResetPose()));

        // ═══ Initialize gate closed ═══
        CommandScheduler.getInstance().schedule(new BlockerCommand(robot, Blocker.BlockerState.BLOCKED));
        waitForStart();

        if(isStarted()){
            robot.follower.startTeleopDrive();
        }

        while (opModeIsActive()) {
            // Drive with gamepad 1
            robot.follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x * BotConstants.turnSpeed, true);

            // Manual turret control from GP2 right stick or GP1 left bumper
            if (Math.abs(gamepad2.right_stick_x) > 0.2) {
                double power = gamepad2.right_stick_x > 0 ? TurretConstants.MANUAL_TURRET_POWER : -TurretConstants.MANUAL_TURRET_POWER;
                robot.turret.setManualControl(true, power);
            } else if (gamepad1.left_bumper) {
                robot.turret.setManualControl(true, -TurretConstants.MANUAL_TURRET_POWER);
            } else {
                robot.turret.setManualControl(false, 0);
            }

            // Turret homing sweep
            robot.turret.commandHomingSweep(gamepad1.dpad_up || gamepad2.left_bumper);

            // Manual intake from triggers
            if (gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1) {
                robot.intake.setState(Intake.IntakeState.REVERSE);
            } else if (gamepad1.right_trigger > 0.1 || gamepad2.right_trigger > 0.1) {
                robot.intake.setState(Intake.IntakeState.ON);
            }

            robot.update();
        }

        robot.stop();
    }
}
