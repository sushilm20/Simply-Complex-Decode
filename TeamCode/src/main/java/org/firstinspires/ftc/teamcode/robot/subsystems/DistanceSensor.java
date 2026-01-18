package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.MyTelem;
import org.firstinspires.ftc.teamcode.utils.constants.DistanceConstants;

public class DistanceSensor implements Subsystem {
    private final RevColorSensorV3 distanceSensor;
    public boolean ballPresent = true;
    private long lastIsPresentTime = 0;
    private double distance;

    public DistanceSensor(RevColorSensorV3 distanceSensor){
        this.distanceSensor = distanceSensor;
    }

    private double getDistanceCM() {
        return distanceSensor.getDistance(DistanceUnit.CM);
    }

    private boolean meetsThreshold() {
        return getDistanceCM() < DistanceConstants.threshold;
    }

    public boolean isPresent() {
        return ballPresent;
    }

    @Override
    public void periodic() {
        long currentTime = Robot.getTime();
        distance = getDistanceCM();
        if (meetsThreshold()) {
            ballPresent = true;
            lastIsPresentTime = currentTime;
        }
        else if ((currentTime - lastIsPresentTime) > DistanceConstants.determinationTime) {
            ballPresent = false;
        }
        MyTelem.addData("Is present", ballPresent);
        MyTelem.addData("Distance", distance);
    }
}
