package org.firstinspires.ftc.teamcode.opmodes.prod;

import com.arcrobotics.ftclib.command.CommandGroupBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.botcommands.MoveToCloseShootCommand;
import org.firstinspires.ftc.teamcode.robot.commands.botcommands.TransferCancelCommand;
import org.firstinspires.ftc.teamcode.robot.commands.botcommands.TransferCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.BlockerCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.IntakeCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.KickerCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.ShooterCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.TurretCommand;
import org.firstinspires.ftc.teamcode.robot.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.MyTelem;

import static org.firstinspires.ftc.teamcode.opmodes.prod.CloseAutoBlue.autoEndPose;
@TeleOp(name = "Teleop Blue", group = "Comp")
public class TeleopBlue extends LinearOpMode {
    @Override
    public void runOpMode() {
        MyTelem.init(telemetry);
        Robot robot = new Robot(hardwareMap, false, "BLUE");
        GamepadEx gp1 = new GamepadEx(gamepad1);
        GamepadEx gp2 = new GamepadEx(gamepad2);

        gp2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenReleased(new IntakeCommand(robot, Intake.IntakeState.OFF));
        gp2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new IntakeCommand(robot, Intake.IntakeState.REV));
        gp2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new IntakeCommand(robot, Intake.IntakeState.ON));
        gp2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenReleased(new IntakeCommand(robot, Intake.IntakeState.OFF));

        Trigger rightTrig = new Trigger(() -> gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5);
        Trigger leftTrig = new Trigger(() -> gp2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5);

        leftTrig.whenActive(new ShooterCommand(robot, Shooter.ShooterState.CLOSE));
        rightTrig.whenActive(new ShooterCommand(robot, Shooter.ShooterState.FAR));

        leftTrig.whenInactive(new ShooterCommand(robot, Shooter.ShooterState.STOP));
        rightTrig.whenInactive(new ShooterCommand(robot, Shooter.ShooterState.STOP));

        gp2.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new ParallelCommandGroup(
                        new KickerCommand(robot, Kicker.KickerState.ON),
                        new BlockerCommand(robot, Blocker.BlockerState.UNBLOCKED)
                )
        );
        gp2.getGamepadButton(GamepadKeys.Button.B).whenReleased(
                new ParallelCommandGroup(
                        new KickerCommand(robot, Kicker.KickerState.OFF),
                        new BlockerCommand(robot, Blocker.BlockerState.BLOCKED)
                ));

        gp2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new TurretCommand(robot, Turret.TurretState.MATH));
        gp2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenReleased(new TurretCommand(robot, Turret.TurretState.FRONT));
        gp2.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new TransferCommand(robot, false, true)
        );

        gp2.getGamepadButton(GamepadKeys.Button.Y).whenReleased(
                new TransferCancelCommand(robot)
        );

        gp2.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new TransferCommand(robot, false, true)
        );
        gp2.getGamepadButton(GamepadKeys.Button.X).whenReleased(
                new TransferCancelCommand(robot)
        );

        gp2.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new ParallelCommandGroup(
                        new KickerCommand(robot, Kicker.KickerState.ON),
                        new BlockerCommand(robot, Blocker.BlockerState.UNBLOCKED),
                        new IntakeCommand(robot, Intake.IntakeState.ON)
                )
        );

        gp2.getGamepadButton(GamepadKeys.Button.A).whenReleased(
                new ParallelCommandGroup(
                        new KickerCommand(robot, Kicker.KickerState.OFF),
                        new BlockerCommand(robot, Blocker.BlockerState.BLOCKED),
                        new IntakeCommand(robot, Intake.IntakeState.OFF)
                )
        );

        gp1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new InstantCommand(robot::holding)
        );

        gp1.getGamepadButton(GamepadKeys.Button.B).whenReleased(
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
            robot.follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            robot.update();
        }

        robot.stop();
    }
}
