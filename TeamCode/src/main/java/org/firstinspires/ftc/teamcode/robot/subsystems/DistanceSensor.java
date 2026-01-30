package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.MyTelem;
import org.firstinspires.ftc.teamcode.utils.constants.DistanceConstants;
import static org.firstinspires.ftc.teamcode.utils.constants.IndexerConstants.greenRatio;
import static org.firstinspires.ftc.teamcode.utils.constants.IndexerConstants.greenThreshold;
import static org.firstinspires.ftc.teamcode.utils.constants.IndexerConstants.purpleBlueThreshhold;
import static org.firstinspires.ftc.teamcode.utils.constants.IndexerConstants.purpleRedThreshold;
import static org.firstinspires.ftc.teamcode.utils.constants.IndexerConstants.purpleRatio;

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

    public String getBallColor(){
        double red = distanceSensor.red();
        double green = distanceSensor.green();
        double blue = distanceSensor.blue();
        boolean isGreen = false;
        boolean isPurple = false;

        

        if (green >  greenThreshold &&
                green > (red * greenRatio) &&
                green > (blue * greenRatio)) {
            isGreen = true;
        }

        if (red > purpleRedThreshold &&
                blue > purpleBlueThreshhold &&
                red > green * purpleRatio &&
                blue > green * purpleRatio) {
            isPurple = true;
        }

        if (isGreen == true){
            return "GREEN";
        } else if (isPurple == true) {
            return "PURPLE";

        } else {
            return "UNKNOWN";
        }

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
