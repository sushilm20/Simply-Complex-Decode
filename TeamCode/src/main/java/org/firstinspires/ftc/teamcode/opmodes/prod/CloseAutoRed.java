package org.firstinspires.ftc.teamcode.opmodes.prod;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.dashboard.config.Config;

@Autonomous(name = "Close Side Auto Red", group = "Prod")
@Config
public class CloseAutoRed extends CloseSideAuto {
    public CloseAutoRed() { super("RED"); }
}


