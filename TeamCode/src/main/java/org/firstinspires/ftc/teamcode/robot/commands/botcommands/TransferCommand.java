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
    public TransferCommand(Robot robot, boolean shortSide, boolean math){
        if (shortSide && !math) {
            addCommands(
                    new ShooterCommand(robot, Shooter.ShooterState.CLOSE),
                    new WaitCommand(blockerWait),
                    new BlockerCommand(robot, Blocker.BlockerState.UNBLOCKED),
                    new WaitCommand(firstBallWaitClose),
                    new KickerCommand(robot, Kicker.KickerState.ON),
                    new IntakeCommand(robot, Intake.IntakeState.ON),
                    new WaitCommand(intakeWait)
            );
        }
        else if(!shortSide && !math) {
            addCommands(
                    new ShooterCommand(robot, Shooter.ShooterState.FAR),
                    new WaitCommand(blockerWait),
                    new BlockerCommand(robot, Blocker.BlockerState.UNBLOCKED),
                    new WaitCommand(firstBallWaitFar),
                    new KickerCommand(robot, Kicker.KickerState.ON),
                    new IntakeCommand(robot, Intake.IntakeState.ON),
                    new WaitCommand(intakeWait)
            );
        } else if(math){
            addCommands(
                    new ShooterCommand(robot, Shooter.ShooterState.MATH),
                    new WaitCommand(blockerWait),
                    new BlockerCommand(robot, Blocker.BlockerState.UNBLOCKED),
                    new KickerCommand(robot, Kicker.KickerState.ON),
                    new WaitCommand(kickerWait),
                    new IntakeCommand(robot, Intake.IntakeState.ON),
                    new WaitCommand(intakeWait)
            );
        }
    }
}
