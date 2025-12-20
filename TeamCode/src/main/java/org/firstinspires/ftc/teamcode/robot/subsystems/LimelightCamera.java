package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.MyTelem;

public class LimelightCamera implements Subsystem {
    public static class TagTarget {
        public boolean hasTarget = false;
        public Pose3D botPose;
        public int id = -1;
        public double tX = 0.0;
        public double tY = 0.0;
        public double tArea = 0.0;
        public double distanceM = 0.0;
    }

    private final Limelight3A limelight;
    private final TagTarget target = new TagTarget();
    private int desiredTagId = -1;
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
        if (result == null || !result.isValid()) {
            target.hasTarget = false;
            target.id = -1;
            target.distanceM = 0.0;
            return;
        }

        target.hasTarget = true;

        target.tX = result.getTx();
        target.tY = result.getTy();
        target.tArea = result.getTa();

        Pose3D botPose = result.getBotpose();
        target.botPose = botPose;

        if (botPose != null) {
            double x = botPose.getPosition().x;
            double y = botPose.getPosition().y;
            target.distanceM = Math.hypot(x, y);
        } else {
            target.distanceM = 0.0;
        }
    }
    private void publishTelem() {
        MyTelem.addData("LL Has Target", target.hasTarget);
        if (!target.hasTarget) return;

        MyTelem.addData("LL Tag ID", target.id);
        MyTelem.addData("LL tX", target.tX);
        MyTelem.addData("LL tArea", target.tArea);
        MyTelem.addData("LL tY", target.tY);
        MyTelem.addData("LL Target Distance", target.distanceM);
        MyTelem.addData("Bot Pose", target.botPose);
    }

}
