package org.firstinspires.ftc.teamcode.robot.commands.botcommands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.BlockerCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.IndexerCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.IntakeCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.ShooterCommand;
import org.firstinspires.ftc.teamcode.robot.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.robot.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;

public class TransferCancelCommand extends SequentialCommandGroup {
    public TransferCancelCommand(Robot robot){
        addCommands(
             new IntakeCommand(robot, Intake.IntakeState.OFF),
             new ShooterCommand(robot, Shooter.ShooterState.STOP),
             new BlockerCommand(robot, Blocker.BlockerState.BLOCKED),
             new IndexerCommand(robot, Indexer.IndexState.IN)  // open claw
        );
    }

    public TransferCancelCommand(Robot robot, Shooter.ShooterState state){
        addCommands(
                new IntakeCommand(robot, Intake.IntakeState.OFF),
                new ShooterCommand(robot, state),
                new BlockerCommand(robot, Blocker.BlockerState.BLOCKED),
                new IndexerCommand(robot, Indexer.IndexState.IN)  // open claw
        );
    }
}
