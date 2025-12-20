package org.firstinspires.ftc.teamcode.opmodes.testers;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.utils.MyTelem;

@Config
@TeleOp(name = "Limelight Tester")
public class LimelightTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MyTelem.init(telemetry);

        Robot robot = new Robot(hardwareMap, false);
        robot.follower.setStartingPose(new Pose(0, 0, 0));
        waitForStart();

        if (isStopRequested()) return;

        robot.follower.startTeleopDrive();

        while (opModeIsActive()) {
            robot.follower.setTeleOpMovementVectors(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true
            );

            robot.update();

            LimelightCamera.TagTarget t = robot.limelightCamera.getTargetTag();

            MyTelem.addData("LL Has Target", t.hasTarget);

            if (t.hasTarget) {
                MyTelem.addData("LL tX", t.tX);
                MyTelem.addData("LL tY", t.tY);
                MyTelem.addData("LL Distance", t.distanceM);
            }

            MyTelem.update();
        }

        robot.stop();
    }
}
