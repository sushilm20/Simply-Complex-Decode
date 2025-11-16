package org.firstinspires.ftc.teamcode.opmodes.prod;

import static org.firstinspires.ftc.teamcode.pedroPathing.constants.AutoConstants.firstIntakeX;
import static org.firstinspires.ftc.teamcode.pedroPathing.constants.AutoConstants.firstWait;
import static org.firstinspires.ftc.teamcode.pedroPathing.constants.AutoConstants.intakeBallWait;
import static org.firstinspires.ftc.teamcode.pedroPathing.constants.AutoConstants.secondIntakeX;
import static org.firstinspires.ftc.teamcode.pedroPathing.constants.AutoConstants.secondWait;
import static org.firstinspires.ftc.teamcode.pedroPathing.constants.AutoConstants.shootingX;
import static org.firstinspires.ftc.teamcode.pedroPathing.constants.AutoConstants.shootingY;
import static org.firstinspires.ftc.teamcode.pedroPathing.constants.AutoConstants.thirdWait;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandGroupBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.botcommands.FollowPathCommand;
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
import org.firstinspires.ftc.teamcode.pedroPathing.constants.CloseSideAutoPoseData;

public class CloseSideAuto extends OpMode {
    public static Pose autoEndPose;
    private ElapsedTime timer;
    private DashboardPoseTracker dashboardPoseTracker;
    private Robot robot;
    private CloseSideAutoPaths paths;
    private SequentialCommandGroup auto;
    private String color;

    public CloseSideAuto(String color) {
        this.color = color;
    }

    public CloseSideAuto() {
        this("BLUE");
    }

    @Override
    public void init() {
        timer = new ElapsedTime();
        MyTelem.init(telemetry);
        robot = new Robot(hardwareMap, true, color);

        CommandScheduler.getInstance().schedule(
                new BlockerCommand(robot, Blocker.BlockerState.BLOCKED),
                new TurretCommand(robot, Turret.TurretState.FRONT));

        CommandGroupBase transferCommand = new SequentialCommandGroup(
                new IntakeCommand(robot, Intake.IntakeState.OFF),
                new BlockerCommand(robot, Blocker.BlockerState.UNBLOCKED),
                new WaitCommand(firstWait),
                new KickerCommand(robot, Kicker.KickerState.ON),
                new WaitCommand(secondWait),
                new IntakeCommand(robot, Intake.IntakeState.ON),
                new WaitCommand(thirdWait),
                new ShooterCommand(robot, Shooter.ShooterState.STOP),
                new KickerCommand(robot, Kicker.KickerState.OFF),
                new BlockerCommand(robot, Blocker.BlockerState.BLOCKED));
        CommandGroupBase shootThree = new SequentialCommandGroup(transferCommand);

        paths = new CloseSideAutoPaths(robot.follower, color);
        auto = new SequentialCommandGroup(
                new FollowPathCommand(robot.follower, paths.Path1),
                new ShooterCommand(robot, Shooter.ShooterState.CLOSEAUTO),
                shootThree,
                new FollowPathCommand(robot.follower, paths.Path2),
                new WaitCommand(intakeBallWait),
                new FollowPathCommand(robot.follower, paths.Path3),
                new ShooterCommand(robot, Shooter.ShooterState.CLOSEAUTO),
                shootThree,
                new FollowPathCommand(robot.follower, paths.Path4),
                new WaitCommand(intakeBallWait),
                new FollowPathCommand(robot.follower, paths.Path5),
                new ShooterCommand(robot, Shooter.ShooterState.CLOSEAUTO),
                shootThree,
                new FollowPathCommand(robot.follower, paths.Path6),
                new WaitCommand(intakeBallWait),
                new FollowPathCommand(robot.follower, paths.Path7),
                new ShooterCommand(robot, Shooter.ShooterState.CLOSEAUTO),
                shootThree,
                new FollowPathCommand(
                        robot.follower,
                        robot.follower.pathBuilder()
                                .addPath(new BezierLine(
                                        new Pose(robot.follower.getPose().getX(), robot.follower.getPose().getY()),
                                        new Pose(mirror(51.9627), 70.1784)))
                                .setConstantHeadingInterpolation(robot.follower.getPose().getHeading())
                                .build()));

        dashboardPoseTracker = new DashboardPoseTracker(robot.follower.poseUpdater);
        Drawing.drawRobot(robot.follower.poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        robot.update();
        MyTelem.addData("POSE", robot.follower.getPose());
        MyTelem.addData("TIMER", timer.seconds());
        MyTelem.update();
        dashboardPoseTracker.update();
        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
        Drawing.drawRobot(robot.follower.poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }

    @Override
    public void start() {
        CommandScheduler.getInstance().schedule(auto);
    }

    private double mirror(double x) {
        return color.equals("RED") ? 144 - x : x;
    }

    private double mirrorHeading(double headingDeg) {
        return color.equals("RED") ? (180 - headingDeg) : headingDeg;
    }

    public static class CloseSideAutoPaths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7;

        public CloseSideAutoPaths(Follower follower, String color) {
            Pose startPose = CloseSideAutoPoseData.mirror(CloseSideAutoPoseData.START_POSE, color);
            double startHeading = CloseSideAutoPoseData.mirrorHeading(CloseSideAutoPoseData.START_HEADING, color);
            Pose shootingPose = CloseSideAutoPoseData.mirror(CloseSideAutoPoseData.SHOOTING_POSE, color);
            Pose mid1Curve = CloseSideAutoPoseData.mirror(CloseSideAutoPoseData.MID1_CURVE, color);
            Pose firstIntake = CloseSideAutoPoseData.mirror(CloseSideAutoPoseData.FIRST_INTAKE, color);
            Pose mid2Curve = CloseSideAutoPoseData.mirror(CloseSideAutoPoseData.MID2_CURVE, color);
            Pose secondIntake = CloseSideAutoPoseData.mirror(CloseSideAutoPoseData.SECOND_INTAKE, color);
            Pose mid3Curve = CloseSideAutoPoseData.mirror(CloseSideAutoPoseData.MID3_CURVE, color);
            Pose finalIntake = CloseSideAutoPoseData.mirror(CloseSideAutoPoseData.FINAL_INTAKE, color);
            Pose trailerFinish = CloseSideAutoPoseData.mirror(CloseSideAutoPoseData.TRAILER_FINISH, color);

            double shootingHeading = CloseSideAutoPoseData.mirrorHeading(CloseSideAutoPoseData.SHOOTING_HEADING, color);
            double heading180 = 180;

            follower.setStartingPose(startPose);
            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, shootingPose))
                    .setLinearHeadingInterpolation(Math.toRadians(startHeading), Math.toRadians(shootingHeading))
                    .build();
            Path2 = follower.pathBuilder()
                    .addPath(new BezierCurve(shootingPose, mid1Curve, firstIntake))
                    .setLinearHeadingInterpolation(Math.toRadians(shootingHeading), Math.toRadians(heading180))
                    .setZeroPowerAccelerationMultiplier(2)
                    .build();
            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(firstIntake, shootingPose))
                    .setLinearHeadingInterpolation(Math.toRadians(heading180), Math.toRadians(shootingHeading))
                    .build();
            Path4 = follower.pathBuilder()
                    .addPath(new BezierCurve(shootingPose, mid2Curve, secondIntake))
                    .setLinearHeadingInterpolation(Math.toRadians(shootingHeading), Math.toRadians(heading180))
                    .setZeroPowerAccelerationMultiplier(2)
                    .build();
            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(secondIntake, shootingPose))
                    .setLinearHeadingInterpolation(Math.toRadians(heading180), Math.toRadians(shootingHeading))
                    .build();
            Path6 = follower.pathBuilder()
                    .addPath(new BezierCurve(shootingPose, mid3Curve, finalIntake))
                    .setLinearHeadingInterpolation(Math.toRadians(shootingHeading), Math.toRadians(heading180))
                    .setZeroPowerAccelerationMultiplier(2)
                    .build();
            Path7 = follower.pathBuilder()
                    .addPath(new BezierLine(trailerFinish, shootingPose))
                    .setLinearHeadingInterpolation(Math.toRadians(heading180), Math.toRadians(shootingHeading))
                    .build();
        }
    }
}
