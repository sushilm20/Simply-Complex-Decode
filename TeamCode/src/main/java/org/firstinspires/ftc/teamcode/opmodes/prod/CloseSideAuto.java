package org.firstinspires.ftc.teamcode.opmodes.prod;

import static org.firstinspires.ftc.teamcode.utils.constants.auto.AutoConstants.leverWait;

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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.botcommands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.robot.commands.botcommands.TransferCancelCommand;
import org.firstinspires.ftc.teamcode.robot.commands.botcommands.TransferCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.BlockerCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.IntakeCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.ShooterCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.TurretCommand;
import org.firstinspires.ftc.teamcode.robot.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.MyTelem;
import org.firstinspires.ftc.teamcode.utils.constants.auto.AutoConstants;
import org.firstinspires.ftc.teamcode.utils.constants.BotConstants;
import org.firstinspires.ftc.teamcode.utils.constants.auto.CloseSideAutoPoseData;
import org.firstinspires.ftc.teamcode.utils.constants.TurretConstants;

public class CloseSideAuto extends OpMode {
    private ElapsedTime timer;
    private DashboardPoseTracker dashboardPoseTracker;
    private Robot robot;
    private CloseSideAutoPaths paths;
    private SequentialCommandGroup auto;
    private String color;

    public CloseSideAuto(String color) {
        this.color = color;
    }

    @Override
    public void init() {
        timer = new ElapsedTime();
        MyTelem.init(telemetry);
        robot = new Robot(hardwareMap, true, color);
        paths = new CloseSideAutoPaths(robot.follower, color);

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                    new BlockerCommand(robot, Blocker.BlockerState.BLOCKED),
                    new TurretCommand(robot, Turret.TurretState.FRONT)
                )
        );

        BotConstants.BotState state = Robot.botState;
        Shooter.ShooterState shooterState =
                state == BotConstants.BotState.MATH ? Shooter.ShooterState.MATH :
                        state == BotConstants.BotState.MANUAL ? Shooter.ShooterState.CLOSE :
                                Shooter.ShooterState.TESTING;

        auto = new SequentialCommandGroup(
                new TurretCommand(robot, Turret.TurretState.MATH_CAMERA),
                new IntakeCommand(robot, Intake.IntakeState.ON),
                new ShooterCommand(robot, shooterState),
                new ParallelCommandGroup(
                    new FollowPathCommand(robot.follower, paths.Path1),
                    shootThree()
                ),
                new IntakeCommand(robot, Intake.IntakeState.ON),
                new ParallelCommandGroup(
                    new TurretCommand(robot, Turret.TurretState.FRONT),
                    new FollowPathCommand(robot.follower, paths.Path2)
                ),
                new WaitCommand(200),
                new ParallelCommandGroup(
                        new FollowPathCommand(robot.follower, paths.Path3),
                        new TurretCommand(robot, Turret.TurretState.MATH_CAMERA),
                        new ShooterCommand(robot, shooterState)
                ),
                shootThree(),
                new ShooterCommand(robot, shooterState),
                new TurretCommand(robot, Turret.TurretState.FRONT),
                new FollowPathCommand(robot.follower, paths.LeverPath),
                new WaitCommand((long) leverWait),
                new IntakeCommand(robot, Intake.IntakeState.ON),
                new FollowPathCommand(robot.follower, paths.LeverIntakePath),
                new WaitCommand(1300),
                new ParallelCommandGroup(
                        new TurretCommand(robot, Turret.TurretState.MATH_CAMERA),
                        new FollowPathCommand(robot.follower, paths.LeverReturnPath)
                ),
                shootThree(),
                new ShooterCommand(robot, shooterState),
                new TurretCommand(robot, Turret.TurretState.FRONT),
                new FollowPathCommand(robot.follower, paths.LeverPath2),
                new WaitCommand((long) leverWait),
                new IntakeCommand(robot, Intake.IntakeState.ON),
                new FollowPathCommand(robot.follower, paths.LeverIntakePath2),
                new WaitCommand(1300),
                new ParallelCommandGroup(
                        new TurretCommand(robot, Turret.TurretState.MATH_CAMERA),
                        new FollowPathCommand(robot.follower, paths.LeverReturnPath2),
                        new SequentialCommandGroup(
                                new WaitCommand(300),
                                new IntakeCommand(robot, Intake.IntakeState.ON)
                        )
                ),
                shootThree(),
                new IntakeCommand(robot, Intake.IntakeState.ON),
                new ParallelCommandGroup(
                    new TurretCommand(robot, Turret.TurretState.FRONT),
                    new FollowPathCommand(robot.follower, paths.Path4)
                ),
                new ParallelCommandGroup(
                    new FollowPathCommand(robot.follower, paths.Path5),
                    new SequentialCommandGroup(
                            new IntakeCommand(robot, Intake.IntakeState.OFF),
                            new WaitCommand(300),
                            new IntakeCommand(robot, Intake.IntakeState.ON)
                    ),
                    new TurretCommand(robot, Turret.TurretState.MATH_CAMERA),
                    new ShooterCommand(robot, shooterState)
                ),
                shootThree(),
                new IntakeCommand(robot, Intake.IntakeState.ON),
                new ParallelCommandGroup(
                    new TurretCommand(robot, Turret.TurretState.FRONT),
                    new FollowPathCommand(robot.follower, paths.Path6)
                ),
                new InstantCommand(() -> TurretConstants.closeTolerance = 10),
                new TurretCommand(robot, Turret.TurretState.MATH_CAMERA),
                new ParallelCommandGroup(
                    new FollowPathCommand(robot.follower, paths.Path7),
                    new SequentialCommandGroup(
                        new IntakeCommand(robot, Intake.IntakeState.OFF),
                        new WaitCommand(300),
                        new IntakeCommand(robot, Intake.IntakeState.ON)
                    ),
                    new ShooterCommand(robot, shooterState)
                ),
                new WaitCommand(200),
                shootThree()
        );

        dashboardPoseTracker = new DashboardPoseTracker(robot.follower.poseUpdater);
        Drawing.drawRobot(robot.follower.poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }

    private CommandGroupBase shootThree() {
        return new SequentialCommandGroup(
                new TransferCommand(robot),
                new TransferCancelCommand(robot),
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
            new SequentialCommandGroup(
                auto,
                new IntakeCommand(robot, Intake.IntakeState.OFF)
            )
        );
    }

    public static class CloseSideAutoPaths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7;
        public PathChain LeverPath, LeverPath2, LeverIntakePath, LeverReturnPath, LeverIntakePath2, LeverReturnPath2;

        public CloseSideAutoPaths(Follower follower, String color) {
            Pose startPose = CloseSideAutoPoseData.mirror(CloseSideAutoPoseData.START_POSE, color);
            double startHeading = CloseSideAutoPoseData.mirrorHeading(CloseSideAutoPoseData.START_HEADING, color);
            Pose shootingPose = CloseSideAutoPoseData.mirror(CloseSideAutoPoseData.SHOOTING_POSE, color);
            Pose finalShootingPose = CloseSideAutoPoseData.mirror(CloseSideAutoPoseData.FINAL_SHOOT, color);
            Pose mid1Curve = CloseSideAutoPoseData.mirror(CloseSideAutoPoseData.MID1_CURVE, color);
            Pose firstIntake = CloseSideAutoPoseData.mirror(CloseSideAutoPoseData.FIRST_INTAKE, color);
            Pose mid2Curve = CloseSideAutoPoseData.mirror(CloseSideAutoPoseData.MID2_CURVE, color);
            Pose secondIntake = CloseSideAutoPoseData.mirror(CloseSideAutoPoseData.SECOND_INTAKE, color);
            Pose mid3Curve = CloseSideAutoPoseData.mirror(CloseSideAutoPoseData.MID3_CURVE, color);
            Pose finalIntake = CloseSideAutoPoseData.mirror(CloseSideAutoPoseData.FINAL_INTAKE, color);
            Pose leverControl = CloseSideAutoPoseData.mirror(CloseSideAutoPoseData.LEVER_CONTROL, color);
            Pose leverPose = CloseSideAutoPoseData.mirror(CloseSideAutoPoseData.LEVER, color);
            Pose leverIntakePose = CloseSideAutoPoseData.mirror(CloseSideAutoPoseData.LEVER_INTAKE, color);
            double heading180 = CloseSideAutoPoseData.mirrorHeading(180, color);
            double leverHeading = CloseSideAutoPoseData.mirrorHeading(AutoConstants.leverHeading, color);
            double finalShootHeading = CloseSideAutoPoseData.mirrorHeading(AutoConstants.finalShootHeading, color);
            follower.setStartingPose(startPose);
            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, shootingPose))
//                    .(Math.toRadians(startHeading), Math.toRadians(heading180))
                    .setTangentHeadingInterpolation()
                    .setReversed(true)
                    .build();
            Path2 = follower.pathBuilder()
                    .addPath(new BezierCurve(shootingPose, mid2Curve, secondIntake))
                    .setLinearHeadingInterpolation(Math.toRadians(heading180), Math.toRadians(heading180))
                    .setZeroPowerAccelerationMultiplier(6)
                    .build();
            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(secondIntake, shootingPose))
                    .setTangentHeadingInterpolation()
                    .setReversed(true)
                    .build();
            LeverPath = follower.pathBuilder()
                    .addPath(new BezierLine(shootingPose, leverPose))
                    .setLinearHeadingInterpolation(Math.toRadians(heading180), Math.toRadians(heading180))
                    .setZeroPowerAccelerationMultiplier(3)
                    .build();
            LeverIntakePath = follower.pathBuilder()
                    .addPath(new BezierCurve(leverPose, leverControl, leverIntakePose))
                    .setLinearHeadingInterpolation(Math.toRadians(heading180), Math.toRadians(leverHeading))
                    .build();
            LeverReturnPath = follower.pathBuilder()
                    .addPath(new BezierLine(leverIntakePose, shootingPose))
                    .setTangentHeadingInterpolation()
                    .setReversed(true)
                    .build();
            LeverPath2 = follower.pathBuilder()
                    .addPath(new BezierLine(shootingPose, leverPose))
                    .setLinearHeadingInterpolation(Math.toRadians(heading180), Math.toRadians(heading180))
                    .setZeroPowerAccelerationMultiplier(3)
                    .build();
            LeverIntakePath2 = follower.pathBuilder()
                    .addPath(new BezierCurve(leverPose, leverControl, leverIntakePose))
                    .setLinearHeadingInterpolation(Math.toRadians(heading180), Math.toRadians(leverHeading))
                    .build();
            LeverReturnPath2 = follower.pathBuilder()
                    .addPath(new BezierLine(leverIntakePose, shootingPose))
                    .setTangentHeadingInterpolation()
                    .setReversed(true)
                    .build();
            Path4 = follower.pathBuilder()
                    .addPath(new BezierCurve(shootingPose, mid1Curve, firstIntake))
                    .setLinearHeadingInterpolation(Math.toRadians(heading180), Math.toRadians(heading180))
                    .build();
            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(firstIntake, shootingPose))
                    .setTangentHeadingInterpolation()
                    .setReversed(true)
                    .build();
            Path6 = follower.pathBuilder()
                    .addPath(new BezierCurve(shootingPose, mid3Curve, finalIntake))
                    .setLinearHeadingInterpolation(Math.toRadians(finalShootHeading), Math.toRadians(heading180))
                    .build();
            Path7 = follower.pathBuilder()
                    .addPath(new BezierLine(finalIntake, finalShootingPose))
                    .setTangentHeadingInterpolation()
                    .setReversed(true)
                    .build();
        }
    }
}
