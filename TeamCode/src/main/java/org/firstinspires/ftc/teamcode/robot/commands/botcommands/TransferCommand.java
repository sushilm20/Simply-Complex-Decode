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
import org.firstinspires.ftc.teamcode.utils.MyTelem;
import org.firstinspires.ftc.teamcode.utils.constants.BotConstants;

@Config
public class TransferCommand extends SequentialCommandGroup {
    public static int speedUpDelay = 0; 
    public static int intakeWait = 0;
    public static int blockerWait = 0;
    public TransferCommand(Robot robot){
        BotConstants.BotState state = Robot.botState;
        MyTelem.addData("Robot State", state);
        Shooter.ShooterState shooterState =
                state == BotConstants.BotState.MATH ? Shooter.ShooterState.MATH :
                        state == BotConstants.BotState.MANUAL ? Shooter.ShooterState.CLOSE :
                                Shooter.ShooterState.TESTING;
        MyTelem.addData("Shooter State", shooterState);

        addCommands(
                new ShooterCommand(robot, shooterState),
                new ParallelRaceGroup(
                    new WaitUntilCommand(() -> robot.shooter.shooterAtRPM()),
                    new WaitCommand(3000)
                ),
                new BlockerCommand(robot, Blocker.BlockerState.UNBLOCKED),
                new WaitCommand(200),
                new KickerCommand(robot, Kicker.KickerState.ON),
                new IntakeCommand(robot, Intake.IntakeState.ON),
                new WaitCommand(1250)
        );
    }


    public enum TransferCommandState{
        CLOSE, FAR, MATH
    }
}
