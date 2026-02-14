package org.firstinspires.ftc.teamcode.opmodes.prod;

import com.arcrobotics.ftclib.command.CommandGroupBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.botcommands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.robot.commands.botcommands.TransferCancelCommand;
import org.firstinspires.ftc.teamcode.robot.commands.botcommands.TransferCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.BlockerCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.IntakeCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.TurretCommand;
import org.firstinspires.ftc.teamcode.robot.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.MyTelem;
import org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.utils.constants.auto.AutoConstants;
import org.firstinspires.ftc.teamcode.utils.constants.auto.CloseSideAutoPoseData;
import org.firstinspires.ftc.teamcode.utils.constants.farauto.FarSideAutoPoseData;

public class FarSideAuto extends OpMode {
    private ElapsedTime timer;
    private DashboardPoseTracker dashboardPoseTracker;
    private Robot robot;
    private FarSideAuto.FarSideAutoPaths paths;
    private SequentialCommandGroup auto;
    private String color;

    public FarSideAuto(String color) {
        this.color = color;
    }

    @Override
    public void init() {
        timer = new ElapsedTime();
        MyTelem.init(telemetry);
        robot = new Robot(hardwareMap, true, color);
        paths = new FarSideAuto.FarSideAutoPaths(robot.follower, color);

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new BlockerCommand(robot, Blocker.BlockerState.BLOCKED),
                        new TurretCommand(robot, Turret.TurretState.MATH)
                )
        );


        auto = new SequentialCommandGroup(
            new TurretCommand(robot, Turret.TurretState.MATH),
            shootThree(),
            new FollowPathCommand(robot.follower, paths.Path1),
            new IntakeCommand(robot, Intake.IntakeState.SOLOFRONT),
            new FollowPathCommand(robot.follower, paths.Path2),
            shootThree(),
            new FollowPathCommand(robot.follower, paths.Path3),
            new IntakeCommand(robot, Intake.IntakeState.SOLOFRONT),
            new FollowPathCommand(robot.follower, paths.Path4),
            shootThree(),
            new FollowPathCommand(robot.follower, paths.Path5),
            new WaitCommand(300),
            new IntakeCommand(robot, Intake.IntakeState.SOLOFRONT),
            new FollowPathCommand(robot.follower, paths.Path6),
            shootThree()
        );

        dashboardPoseTracker = new DashboardPoseTracker(robot.follower.poseUpdater);
        Drawing.drawRobot(robot.follower.poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }

    private CommandGroupBase shootThree() {
        return new SequentialCommandGroup(
                new TransferCommand(robot),
                new WaitCommand(300),
                new TransferCancelCommand(robot, Shooter.ShooterState.SPEEDING_UP),
                new IntakeCommand(robot, Intake.IntakeState.ON)
        );
    }

    @Override
    public void loop() {
        MyTelem.addData("POSE", robot.follower.getPose());
        MyTelem.addData("TIMER", timer.seconds());
        robot.update();
        dashboardPoseTracker.update();
        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
        Drawing.drawRobot(robot.follower.poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }

    @Override
    public void start() {
        timer.reset();
        CommandScheduler.getInstance().schedule(
                auto
        );
    }

    public static class FarSideAutoPaths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6;

        public FarSideAutoPaths(Follower follower, String color) {
            Pose startPose = FarSideAutoPoseData.mirror(FarSideAutoPoseData.START_POSE, color);
            Pose mid1Curve = FarSideAutoPoseData.mirror(FarSideAutoPoseData.MID1_CURVE, color);
            Pose firstTarget = FarSideAutoPoseData.mirror(FarSideAutoPoseData.FIRST_TARGET, color);
            Pose secondTarget = FarSideAutoPoseData.mirror(FarSideAutoPoseData.SECOND_TARGET, color);
            Pose thirdTarget = FarSideAutoPoseData.mirror(FarSideAutoPoseData.THIRD_TARGET, color);
            double heading90 = FarSideAutoPoseData.mirrorHeading(90, color);
            double heading180 = FarSideAutoPoseData.mirrorHeading(180, color);
            follower.setStartingPose(startPose);

            Path1 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            startPose,
                            mid1Curve,
                            firstTarget
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(heading90),
                            Math.toRadians(heading180)
                    )
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            firstTarget,
                            secondTarget
                    ))
                    .setTangentHeadingInterpolation()
                    .setReversed(true)
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            secondTarget,
                            thirdTarget
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            thirdTarget,
                            secondTarget
                    ))
                    .setTangentHeadingInterpolation()
                    .setReversed(true)
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            secondTarget,
                            thirdTarget
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            thirdTarget,
                            secondTarget
                    ))
                    .setTangentHeadingInterpolation()
                    .setReversed(true)
                    .build();
        }
    }

}
