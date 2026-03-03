package org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class IndexerCommand extends InstantCommand {
    public IndexerCommand(Robot robot, Indexer.IndexState state) {
        super(() -> robot.indexer.setState(state));
    }
}
