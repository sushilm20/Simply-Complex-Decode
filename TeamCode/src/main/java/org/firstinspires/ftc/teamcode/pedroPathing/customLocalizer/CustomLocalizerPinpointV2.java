package org.firstinspires.ftc.teamcode.pedroPathing.customLocalizer;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Vector;

public class CustomLocalizerPinpointV2 extends Localizer {
    @Override
    public Pose getPose() {
        return null;
    }

    @Override
    public Pose getVelocity() {
        return null;
    }

    @Override
    public Vector getVelocityVector() {
        return null;
    }

    @Override
    public void setStartPose(Pose pose) {

    }

    @Override
    public void setPose(Pose pose) {

    }

    @Override
    public void update() {

    }

    @Override
    public double getTotalHeading() {
        return 0;
    }

    @Override
    public double getForwardMultiplier() {
        return 0;
    }

    @Override
    public double getLateralMultiplier() {
        return 0;
    }

    @Override
    public double getTurningMultiplier() {
        return 0;
    }

    @Override
    public void resetIMU() throws InterruptedException {

    }

    @Override
    public boolean isNAN() {
        return false;
    }
}
