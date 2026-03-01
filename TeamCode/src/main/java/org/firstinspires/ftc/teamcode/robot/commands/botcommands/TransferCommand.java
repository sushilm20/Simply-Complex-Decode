package org.firstinspires.ftc.teamcode.robot.commands.botcommands;

import static org.firstinspires.ftc.teamcode.robot.subsystems.Intake.IntakeState.OFF;
import static org.firstinspires.ftc.teamcode.robot.subsystems.Intake.IntakeState.ON;
import static org.firstinspires.ftc.teamcode.robot.subsystems.Intake.IntakeState.SOLOFRONT;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.BlockerCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.IndexerCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.IntakeCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.ShooterCommand;
import org.firstinspires.ftc.teamcode.robot.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.robot.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.utils.MyTelem;
import org.firstinspires.ftc.teamcode.utils.constants.BotConstants;

@Config
public class TransferCommand extends SequentialCommandGroup {
    public TransferCommand(Robot robot, Shooter.ShooterState shooterState) {
        addCommands(
                // Start intake forward
                new IntakeCommand(robot, SOLOFRONT),
                // Spin up shooter to target RPM
                new ShooterCommand(robot, shooterState),
                // Wait until shooter is at RPM (or 3s timeout)
                new ParallelRaceGroup(
                        new WaitUntilCommand(() -> robot.shooter.shooterAtRPM()),
                        new WaitCommand(3000)
                ),
                // Open gate to release ball
                new BlockerCommand(robot, Blocker.BlockerState.UNBLOCKED),
                new WaitCommand(200),
                // Run intake to push ball through
                new IntakeCommand(robot, ON),
                // Trigger claw close (timed — will auto-reopen)
                new IndexerCommand(robot, Indexer.IndexState.OUT),
                new WaitCommand(550)
        );
    }
    public TransferCommand(Robot robot){
        this(robot, determineShooterState(robot));
    }

    private static Shooter.ShooterState determineShooterState(Robot robot) {
        BotConstants.BotState state = robot.botState;
        return state == BotConstants.BotState.MATH ? Shooter.ShooterState.MATH :
                        state == BotConstants.BotState.MANUAL ? Shooter.ShooterState.CLOSE :
                                Shooter.ShooterState.TESTING;
    }


    public enum TransferCommandState{
        CLOSE, FAR, MATH
    }
}
