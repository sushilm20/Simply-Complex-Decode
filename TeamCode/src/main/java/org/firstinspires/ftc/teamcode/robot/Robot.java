package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.utils.constants.BotConstants.goalX;
import static org.firstinspires.ftc.teamcode.utils.constants.BotConstants.goalY;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Point;
import com.pedropathing.pathgen.Vector;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
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
import org.firstinspires.ftc.teamcode.robot.subsystems.DistanceSensor;
import org.firstinspires.ftc.teamcode.robot.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.robot.subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.MyTelem;
import org.firstinspires.ftc.teamcode.utils.constants.BotConstants;
import org.firstinspires.ftc.teamcode.utils.constants.ShooterMathConstants;
import org.firstinspires.ftc.teamcode.utils.constants.TurretConstants;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
public class  Robot {
    public static BotConstants.BotState botState = BotConstants.BotState.MATH;
    public static boolean auto = false;
    public static Vector velocity = new Vector(new Point(0,0));
    public static Pose currentPose = new Pose(0,0,0);
    public boolean holding;
    public static boolean red;
    public static double voltage = 12;

    public static Follower follower;
    private static ElapsedTime timer;
    private double previousVoltageTime;
    DcMotorEx backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor;
    DcMotorEx topShooterMotor, bottomShooterMotor, counterRoller;
    DcMotorEx intakeMotor, intakeMotor2;
    Servo hoodServo, indexServo;
    Servo turretLeftServo, turretRightServo, blockerServo;
    CRServo kickerRightServo, kickerLeftServo;
    public Indexer indexer;
    // all subsystem classes
    public List<LynxModule> hubs;
    public VoltageSensor voltageSensor;
    public Intake intake;
    public Shooter shooter;
    public Turret turret;
    public Kicker kicker;
    public Blocker blocker;

//    public RevColorSensorV3 distanceSensor;
    public static LimelightCamera limelightCamera;
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
        indexServo = hm.get(Servo.class, "indexServo");
        topShooterMotor = hm.get(DcMotorEx.class, "topShooter");
        bottomShooterMotor = hm.get(DcMotorEx.class, "counterRoller");
        hoodServo = hm.get(Servo.class, "hoodServo");
        intakeMotor = hm.get(DcMotorEx.class, "intake");
        intakeMotor2 = hm.get(DcMotorEx.class, "intake2");
        turretLeftServo = hm.get(Servo.class, "turretLeftServo");
        turretRightServo = hm.get(Servo.class, "turretRightServo");
        turretLeftServo.setDirection(Servo.Direction.REVERSE);
        turretRightServo.setDirection(Servo.Direction.REVERSE);
        blockerServo = hm.get(Servo.class, "BlockerServo");
//        distanceSensor = hm.get(RevColorSensorV3.class, "distanceSensor");
//        kickerRightServo = hm.get(CRServo.class, "kickerRightServo");
//        kickerLeftServo = hm.get(CRServo.class, "kickerLeftServo");

        bottomShooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topShooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        bottomShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topShooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        bottomShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        if(!auto){
            if(currentPose != null)
                follower.setStartingPose(currentPose);
            else
                follower.setStartingPose(new Pose(0,0,0));
        }

        intake = new Intake(intakeMotor, intakeMotor2);
        shooter = new Shooter(topShooterMotor, bottomShooterMotor, hoodServo);
        turret = new Turret(turretLeftServo, turretRightServo);
//        kicker = new Kicker(kickerRightServo, kickerLeftServo);
        indexer = new Indexer(indexServo);
        blocker = new Blocker(blockerServo);
//        distSensor = new DistanceSensor(distanceSensor);
//        limelightCamera = new LimelightCamera(llHw, red ? 0 : 1);

        CommandScheduler.getInstance().registerSubsystem(intake, shooter, turret, indexer, blocker);

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
        MyTelem.addData("Velocity", velocity.getMagnitude());
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
    public static double getDistanceFromGoalLL(Pose pose){
        if (getTargetTag().hasTarget && getTargetTag().distance > 0) {
            return getTargetTag().distance;
        } else{
            return getDistanceFromGoal(pose);
        }
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

    public static Pose getGoalPoseLong(){
        if(red){
            return new Pose(144, 144);
        }
        else{
            return new Pose(0, 144);
        }
    }

    public static LimelightCamera.TagTarget getTargetTag(){
        return limelightCamera.getTargetTag();
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

    public static double getShooterMathRPM() {
        return shooterAndTurretMath()[1];
    }

    public static double getTurretAngle() {
        return shooterAndTurretMath()[2];
    }

    public static double getHoodAngle() {
        return shooterAndTurretMath()[0];
    }

    private static Vector getRobotToGoalVector() {
        Pose goalPose = getGoalPose();
        Pose robotPose = Robot.currentPose;
        Vector vector = new Vector(0, 0);
        vector.setOrthogonalComponents(goalPose.getX() - robotPose.getX(), goalPose.getY() - robotPose.getY());
        return vector;
    }
    private static Vector getRobotToGoalVectorTurret() {
        Pose goalPose = getGoalPoseLong();
        Pose robotPose = Robot.currentPose;
        Vector vector = new Vector(0, 0);
        vector.setOrthogonalComponents(goalPose.getX() - robotPose.getX(), goalPose.getY() - robotPose.getY());
        return vector;
    }
    public static long getTime(){
        return timer.time(TimeUnit.MILLISECONDS);
    }

    private static double[] lastGood = new double[]{
            ShooterMathConstants.HOOD_MIN_ANGLE,
            0.0,                                  // flywheel speed
            0.0                                   // turret angle
    };

    private static boolean isFinite(double v) {
        return !Double.isNaN(v) && !Double.isInfinite(v);
    }

    private static double[] shooterAndTurretMath() {
        Vector robotToGoalVector = getRobotToGoalVector();

        double g = 32.174 * 12;
        double dist = robotToGoalVector.getMagnitude();
        double x = dist - ShooterMathConstants.PASS_THROUGH_POINT_RADIUS;
        double y = ShooterMathConstants.SCORE_HEIGHT;
        double a = ShooterMathConstants.SCORE_ANGLE;
        MyTelem.addData("X", x);
        MyTelem.addData("Y", y);
        MyTelem.addData("a", a);
        MyTelem.addData("robotToGoalVector", "(" + robotToGoalVector.getXComponent() + ", " + robotToGoalVector.getYComponent() + ", " + robotToGoalVector.getTheta() + ")");

        if (!isFinite(dist) || !isFinite(x) || x <= 1e-6) {
            MyTelem.addData("SM.guard", "bad dist/x");
            return lastGood;
        }

        double hoodAngle = Math.atan(2 * y / x - Math.tan(a));
        hoodAngle = MathFunctions.clamp(hoodAngle,
                ShooterMathConstants.HOOD_MIN_ANGLE,
                ShooterMathConstants.HOOD_MAX_ANGLE);
        MyTelem.addData("HOOD ANGLE RAW (no vel comp)", hoodAngle);
        double cos = Math.cos(hoodAngle);
        double term = x * Math.tan(hoodAngle) - y;
        double denom1 = 2 * cos * cos * term;

        if (!isFinite(denom1) || denom1 <= 1e-9) {
            MyTelem.addData("SM.guard", "bad denom1");
            return lastGood;
        }

        double flyWheelSpeed = Math.sqrt(g * x * x / denom1);
        if (!isFinite(flyWheelSpeed)) {
            MyTelem.addData("SM.guard", "bad fly1");
            return lastGood;
        }

        Vector robotVelocity = Robot.velocity;

        double rvMag = robotVelocity.getMagnitude();
        double rvTheta = robotVelocity.getTheta();
        double goalTheta = getRobotToGoalVectorTurret().getTheta();

        if (!isFinite(rvMag) || !isFinite(rvTheta) || !isFinite(goalTheta)) {
            MyTelem.addData("SM.guard", "bad vel/theta");
            return lastGood;
        }
        double coordinateTheta = rvTheta - goalTheta;
        double parallelComponent = -Math.cos(coordinateTheta) * rvMag;
        double perpendicularComponent = Math.sin(coordinateTheta) * rvMag;
        MyTelem.addData("RVMag", rvMag);
        MyTelem.addData("RVTheta", rvTheta);
        MyTelem.addData("goalTheta", goalTheta);
        MyTelem.addData("parallel Component", parallelComponent);
        double vz = flyWheelSpeed * Math.sin(hoodAngle);
        double denomT = flyWheelSpeed * Math.cos(hoodAngle);
        if (!isFinite(denomT) || Math.abs(denomT) <= 1e-9) {
            MyTelem.addData("SM.guard", "bad time denom");
            return lastGood;
        }
        double time = x / denomT;
        perpendicularComponent *= ShooterMathConstants.perpMultiplier;
        double ivr = x / time + parallelComponent;
        double nvr = Math.sqrt(ivr * ivr + perpendicularComponent * perpendicularComponent);

        if (!isFinite(nvr) || nvr <= 1e-9) {
            MyTelem.addData("SM.guard", "bad nvr");
            return lastGood;
        }

        double ndr = nvr * time;
        MyTelem.addData("VZ", vz);
        MyTelem.addData("NVR", nvr);
        hoodAngle = Math.max(ShooterMathConstants.HOOD_MIN_ANGLE, Math.min(ShooterMathConstants.HOOD_MAX_ANGLE, Math.atan2(vz, nvr)));

        double cos2 = Math.cos(hoodAngle);
        double denom2 = 2.0 * cos2 * cos2 * (ndr * Math.tan(hoodAngle) - y);
        if (!isFinite(denom2) || denom2 <= 1e-9 || !isFinite(ndr)) {
            MyTelem.addData("SM.guard", "bad denom2");
            return lastGood;
        }

        flyWheelSpeed = Math.sqrt(g * ndr * ndr / denom2);
        if (!isFinite(flyWheelSpeed)) {
            MyTelem.addData("SM.guard", "bad fly2");
            return lastGood;
        }

        double turretVelCompOffset = Math.atan2(perpendicularComponent, ivr) * ShooterMathConstants.perpMultiplier;

        double turretAngle = Math.toDegrees(
                Robot.currentPose.getHeading() - goalTheta + turretVelCompOffset
        );
        if (turretAngle < 0) turretAngle += 360;
        MyTelem.addData("FINAL TURRET", turretAngle);
        MyTelem.addData("PERPENDICULAR COMPONENET", perpendicularComponent);
        MyTelem.addData("IVR", ivr);
        MyTelem.addData("FINAL VELOCITY", flyWheelSpeed);
        MyTelem.addData("TURRET VEL COMP", turretVelCompOffset);
        double[] out = new double[]{hoodAngle, flyWheelSpeed, turretAngle};

        if (isFinite(out[0]) && isFinite(out[1]) && isFinite(out[2])) {
            lastGood = out;
        } else {
            MyTelem.addData("SM.guard", "bad output");
            return lastGood;
        }

        return out;
    }

}
