package org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands;

import com.arcrobotics.ftclib.command.CommandGroupBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Lights;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;

public class LightsCommand extends SequentialCommandGroup {
    public LightsCommand(Robot robot, Lights.LedState state){
        addCommands(
                new InstantCommand(() -> robot.lights.setState(state))
        );
    }
}
