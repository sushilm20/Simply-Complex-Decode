package org.firstinspires.ftc.teamcode.robot.commands.botcommands;

import static org.firstinspires.ftc.teamcode.utils.constants.auto.AutoConstants.shootingX;
import static org.firstinspires.ftc.teamcode.utils.constants.auto.AutoConstants.shootingY;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class MoveToCloseShootCommand extends SequentialCommandGroup {
    public MoveToCloseShootCommand(Robot robot){
        Pose currentPose = robot.follower.getPose();
        Pose finalPose = new Pose(0,0,0);
        double finalHeading = 0;
        if (robot.red){
            finalPose = new Pose(144 - shootingX, shootingY);
            finalHeading = Math.toRadians(180 - 135);
        }
        else{
            finalPose = new Pose(shootingX, shootingY);
            finalHeading = Math.toRadians(135);
        }
        PathChain moveToClose = robot.follower.pathBuilder()
        .addPath(
                new BezierLine(
                        currentPose, finalPose
                )
        ).setLinearHeadingInterpolation(currentPose.getHeading(), Math.toRadians(finalHeading)).build();
        robot.follower.breakFollowing();
        addCommands(
                new WaitCommand(100),
                new FollowPathCommand(robot.follower, moveToClose),
                new InstantCommand(robot::stopHolding)
        );
    }
}
