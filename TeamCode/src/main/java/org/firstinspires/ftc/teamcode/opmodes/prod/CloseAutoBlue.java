package org.firstinspires.ftc.teamcode.opmodes.prod;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.dashboard.config.Config;

@Autonomous(name = "Close Side Auto Blue", group = "Prod")
@Config
public class CloseAutoBlue extends CloseSideAuto {
    public CloseAutoBlue() { super("BLUE"); }
}
