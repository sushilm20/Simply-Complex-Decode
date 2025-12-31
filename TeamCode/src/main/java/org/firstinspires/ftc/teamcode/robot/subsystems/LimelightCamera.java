package org.firstinspires.ftc.teamcode.robot.subsystems;

import static org.firstinspires.ftc.teamcode.utils.constants.BotConstants.cameraYaw;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.MyTelem;
import org.firstinspires.ftc.teamcode.utils.constants.BotConstants;

public class LimelightCamera implements Subsystem {
    private double lastTy = 0.0;
    private double lastTx = 0.0;
    private long lastSeenTimeMs = 0;
    private static final long TARGET_HOLD_MS = 250;
    public static class TagTarget {
        public boolean hasTarget = false;
        public Pose3D botPose;
        public int id = -1;
        public double tX = 0.0;
        public double tY = 0.0;
        public double distance = 0.0;
    }

    private final Limelight3A limelight;
    private final TagTarget target = new TagTarget();
    public LimelightCamera(Limelight3A limelight, int pipelineIndex) {
        this.limelight = limelight;

        limelight.pipelineSwitch(pipelineIndex);
        limelight.start(); //i could do on start
    }

    public TagTarget getTargetTag() {return target;}
    @Override
    public void periodic() {
        updateTarget();
        publishTelem();
    }

    private void updateTarget() {
        LLResult result = limelight.getLatestResult();
        long now = Robot.getTime();
        if (result != null && result.isValid()) {
            target.hasTarget = true;
            target.tX = result.getTx();
            target.tY = result.getTy();
            lastTx = target.tX;
            lastTy = target.tY;

            lastSeenTimeMs = now;
            double totalPitchDeg = target.tY + cameraYaw;

            target.distance = BotConstants.goalDY / Math.tan(Math.toRadians(totalPitchDeg));
            return;
        }

        if (now - lastSeenTimeMs <= TARGET_HOLD_MS) {
            target.hasTarget = true;
            target.tX = lastTx;
            target.tY = lastTy;
            return;
        }
        target.hasTarget = false;
        target.id = -1;
        target.distance = 0.0;
    }
    private void publishTelem() {
        MyTelem.addData("LL Has Target", target.hasTarget);
        if (!target.hasTarget) return;

        MyTelem.addData("LL Tag ID", target.id);
        MyTelem.addData("LL tX", target.tX);
        MyTelem.addData("LL tY", target.tY);
        MyTelem.addData("LL Target Distance", target.distance);
    }

}
