package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.opmodes.testers.GoBildaPrism;
import org.firstinspires.ftc.teamcode.utils.Prism.Color;
import org.firstinspires.ftc.teamcode.utils.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.utils.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.utils.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.utils.constants.LedConstants;

public class Lights implements Subsystem {
    GoBildaPrismDriver prism;

    PrismAnimations.Solid solid = new PrismAnimations.Solid(Color.BLUE);
    PrismAnimations.RainbowSnakes rainbowSnakes = new PrismAnimations.RainbowSnakes();



    public Lights(GoBildaPrismDriver prism){
        this.prism = prism;
        solid.setBrightness(LedConstants.solidBrightness);
        solid.setStartIndex(LedConstants.startIndex);
        solid.setStopIndex(LedConstants.stopIndex);

        rainbowSnakes.setNumberOfSnakes(LedConstants.snakeNumber);
        rainbowSnakes.setSnakeLength(LedConstants.snakeLength);
        rainbowSnakes.setSpacingBetween(LedConstants.snakeSpacing);
        rainbowSnakes.setSpeed(LedConstants.animationSpeed);
    }

    LedState state;

    public void setState(LedState state){
        this.state = state;
        switch (state){
            case BLUE:
                prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, solid);
                break;
            case RAINBOW:
                prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_1, rainbowSnakes);


        }
    }

    public LedState getState(){
        return state;
    }
    public enum LedState{
        BLUE, RAINBOW
    }

}
