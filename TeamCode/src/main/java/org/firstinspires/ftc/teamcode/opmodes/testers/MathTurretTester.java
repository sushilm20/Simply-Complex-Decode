package org.firstinspires.ftc.teamcode.opmodes.testers;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.TurretCommand;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.MyTelem;

@TeleOp
public class MathTurretTester extends LinearOpMode {
    public void runOpMode(){
        MyTelem.init(telemetry);
        Robot robot = new Robot(hardwareMap, false);
        waitForStart();

        GamepadEx gp1 = new GamepadEx(gamepad1);
        gp1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new TurretCommand(robot, Turret.TurretState.MATH)
        );

        gp1.getGamepadButton(GamepadKeys.Button.A).whenReleased(
                new TurretCommand(robot, Turret.TurretState.FRONT)
        );

        while(opModeIsActive()){
            robot.update();
        }
        robot.stop();
    }

}
