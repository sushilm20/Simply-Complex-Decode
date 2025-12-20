package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.utils.constants.BotConstants.goalX;
import static org.firstinspires.ftc.teamcode.utils.constants.BotConstants.goalY;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Point;
import com.pedropathing.pathgen.Vector;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.robot.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.robot.subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.MyTelem;
import org.firstinspires.ftc.teamcode.utils.constants.BotConstants;
import org.firstinspires.ftc.teamcode.utils.constants.TurretConstants;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
public class  Robot {
    public static BotConstants.BotState botState = BotConstants.BotState.MATH;
    public boolean auto = false;
    public static Vector velocity = new Vector(new Point(0,0));
    public static Pose currentPose = new Pose(0,0,0);
    public boolean holding;
    public static boolean red;
    public static double voltage = 12;

    public Follower follower;
    private ElapsedTime timer;
    private double previousVoltageTime;
    DcMotorEx backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor;
    DcMotorEx topShooterMotor, bottomShooterMotor, counterRoller;
    DcMotorEx intakeMotor, intakeMotor2;
    Servo hoodServo;
    Servo turretLeftServo, turretRightServo, blockerServo;
    CRServo kickerRightServo, kickerLeftServo;

    // all subsystem classes
    public List<LynxModule> hubs;
    public VoltageSensor voltageSensor;
    public Intake intake;
    public Shooter shooter;
    public Turret turret;
    public Kicker kicker;
    public Blocker blocker;
    public LimelightCamera limelightCamera;
    public Robot(HardwareMap hm, boolean isAuto, String color){
        this(hm, isAuto);
        red = color.equals("RED");
    }
    public Robot (HardwareMap hm, boolean isAuto) {
        timer = new ElapsedTime();
        timer.reset();
        voltageSensor = hm.voltageSensor.iterator().next();
        previousVoltageTime = timer.time(TimeUnit.MILLISECONDS);
        CommandScheduler.getInstance().reset();

        auto = isAuto;
        TurretConstants.OFFSET = 0.5;

        follower = new Follower(hm, FConstants.class, LConstants.class);

        topShooterMotor = hm.get(DcMotorEx.class, "topShooter");
        bottomShooterMotor = hm.get(DcMotorEx.class, "counterRoller");

        intakeMotor = hm.get(DcMotorEx.class, "intake");
        intakeMotor2 = hm.get(DcMotorEx.class, "intake2");
        turretLeftServo = hm.get(Servo.class, "turretLeftServo");
        turretRightServo = hm.get(Servo.class, "turretRightServo");
        blockerServo = hm.get(Servo.class, "BlockerServo");
        kickerRightServo = hm.get(CRServo.class, "kickerRightServo");
        kickerLeftServo = hm.get(CRServo.class, "kickerLeftServo");

        bottomShooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topShooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        bottomShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topShooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        Limelight3A llHw = hm.get(Limelight3A.class, "limelight");
        bottomShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        if(!auto){
            if(currentPose != null)
                follower.setStartingPose(currentPose);
            else
                follower.setStartingPose(new Pose(0,0,0));
        }

        intake = new Intake(intakeMotor, intakeMotor2);
        shooter = new Shooter(topShooterMotor, bottomShooterMotor);
        turret = new Turret(turretLeftServo, turretRightServo);
        kicker = new Kicker(kickerRightServo, kickerLeftServo);
        blocker = new Blocker(blockerServo);
        limelightCamera = new LimelightCamera(llHw, 0);

        CommandScheduler.getInstance().registerSubsystem(intake, shooter, turret, kicker, blocker, limelightCamera);

        hubs = hm.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        voltage = hm.voltageSensor.iterator().next().getVoltage();
    }

    public void update(){
        CommandScheduler.getInstance().run();
        follower.update();

        if(intake != null)
            MyTelem.addData("Intake State", intake.getState());
        if(shooter != null)
            MyTelem.addData("Shooter State", shooter.getState());
        if(turret != null)
            MyTelem.addData("Turret State", turret.getState());
        if(kicker != null)
            MyTelem.addData("Kicker State", kicker.getState());

        for(LynxModule hub : hubs){
            hub.clearBulkCache();
        }

        if(!holding)
            currentPose = follower.getPose();

        if(auto)
            currentPose = follower.getPose();

        if(timer.time(TimeUnit.MILLISECONDS) - previousVoltageTime > 1000){
            previousVoltageTime = timer.time(TimeUnit.MILLISECONDS);
            voltage = voltageSensor.getVoltage();
        }

        velocity = follower.getVelocity();
        MyTelem.addData("distance from goal", getDistanceFromGoal());
        MyTelem.addData("Current Pose", currentPose);

        MyTelem.addData("Intake Motor Current", intakeMotor2.getCurrent(CurrentUnit.AMPS));
        MyTelem.addData("Shooter Motor Current", topShooterMotor.getCurrent(CurrentUnit.AMPS));

        MyTelem.update();
    }

    public void stop(){
        Pose pose = follower.getPose();
        CommandScheduler.getInstance().reset();
        for(LynxModule hub : hubs){
            hub.clearBulkCache();
        }
        Robot.currentPose = pose;
    }

    public void holding(){
        follower.holdPoint(currentPose);
        holding = true;
    }

    public void stopHolding(){
        follower.breakFollowing();
        follower.startTeleopDrive();
        follower.setMaxPower(1);
        holding = false;
    }
    public static double getDistanceFromGoal(Pose pose){
        Pose goalPose = Robot.getGoalPose();

        double dX = pose.getX() - goalPose.getX();
        double dY = pose.getY() - goalPose.getY();

        return Math.hypot(dX, dY);
    }
    public static double getDistanceFromGoal(){
        return getDistanceFromGoal(Robot.currentPose);
    }
    public static Pose getGoalPose(){
        if(red){
            return new Pose(goalX, goalY);
        }
        else{
            return new Pose(144 - goalX, goalY)
;        }
    }

    public static Pose getEffectiveCoordinates(){
        double distance = Robot.getDistanceFromGoal();
        Vector velocity = Robot.velocity;
        double v_ball = 0;
        double t = v_ball != 0 ? (distance / v_ball) : 0;
        Pose current = Robot.currentPose;
        double eff_x = current.getX();
        double eff_y = current.getY();
        return new Pose(eff_x, eff_y, current.getHeading());
    }
}
