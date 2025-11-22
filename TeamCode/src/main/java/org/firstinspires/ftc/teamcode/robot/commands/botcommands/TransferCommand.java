package org.firstinspires.ftc.teamcode.robot.commands.botcommands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.BlockerCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.IntakeCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.KickerCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.ShooterCommand;
import org.firstinspires.ftc.teamcode.robot.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
@Config
public class TransferCommand extends SequentialCommandGroup {
    public static int firstBallWaitClose = 600;
    public static int firstBallWaitFar = 1000;
    public static int kickerWait = 200;
    public static int intakeWait = 1800;
    public static int blockerWait = 700;
    public TransferCommand(Robot robot, TransferCommandState state){
        if (state == TransferCommandState.CLOSE) {
            addCommands(
                    new ShooterCommand(robot, Shooter.ShooterState.CLOSE),
                    new IntakeCommand(robot, Intake.IntakeState.ON),
                    new WaitCommand(blockerWait),
                    new BlockerCommand(robot, Blocker.BlockerState.UNBLOCKED),
                    new IntakeCommand(robot, Intake.IntakeState.SOLOBACK),
                    new KickerCommand(robot, Kicker.KickerState.ON),
                    new WaitCommand(blockerWait / 2),
                    new WaitCommand(kickerWait),
                    new IntakeCommand(robot, Intake.IntakeState.SOLOBACK),
                    new WaitCommand(intakeWait)
            );
        }
        else if(state == TransferCommandState.FAR) {
            long commandWait = (long) (blockerWait + Math.max(0, ((Robot.getDistanceFromGoal() - 100) * 15)));

            addCommands(
                    new ShooterCommand(robot, Shooter.ShooterState.FAR),
                    new WaitCommand(commandWait / 2),
                    new BlockerCommand(robot, Blocker.BlockerState.UNBLOCKED),
                    new WaitCommand(commandWait / 2),
                    new KickerCommand(robot, Kicker.KickerState.ON),
                    new WaitCommand(kickerWait),
                    new IntakeCommand(robot, Intake.IntakeState.SOLOFRONT),
                    new WaitCommand(intakeWait)
            );
        } else if(state == TransferCommandState.MATH){
            long commandWait = (long) (blockerWait + Math.max(0, ((Robot.getDistanceFromGoal() - 100) * 15)));

            addCommands(
                    new ShooterCommand(robot, Shooter.ShooterState.MATH),
                    new IntakeCommand(robot, Intake.IntakeState.ON),
                    new WaitCommand(commandWait),
                    new BlockerCommand(robot, Blocker.BlockerState.UNBLOCKED),
                    new IntakeCommand(robot, Intake.IntakeState.SOLOBACK),
                    new KickerCommand(robot, Kicker.KickerState.ON),
                    new WaitCommand(commandWait / 2),
                    new WaitCommand(kickerWait),
                    new IntakeCommand(robot, Intake.IntakeState.SOLOBACK),
                    new WaitCommand(intakeWait)
            );
        }
    }

    public enum TransferCommandState{
        CLOSE, FAR, MATH
    }
}
