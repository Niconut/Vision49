package org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_commands;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.PostStorage;
import org.firstinspires.ftc.teamcode.subsystems.robot.MyRobot;
import org.firstinspires.ftc.teamcode.subsystems.scoring.Shooter_Subsystem;

import org.firstinspires.ftc.teamcode.subsystems.Constants.FinalAutoTrajectories;

import static java.lang.Math.PI;
import static java.lang.Math.atan2;

import java.util.function.DoubleSupplier;

@Configurable
public class AutoAimTurretCommand extends CommandBase {
    private MyRobot robot;
    private GoBildaPinpointDriver odo;
    private AnalogInput turretAnalog;

    private PIDController limelight_PID;
    private PIDController odo_PID;

    private final DoubleSupplier turretSupplier;
    private final Shooter_Subsystem scoringShooterSubsystem;
    public final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();


    public static double height1 = 16;// height of limelight lens from the floort
    public static double height2 = 29.5;// height of center of apriltag to the floor
    public static double angle1 = 0; // mounting angle of the limelight degrees back from vertical
    public double distance = 0;
    public double shooterPower = 0;
    private static double turretProp[] = {0, 0};
    private static double turretBearing = 0;
    private static double turretRange = 0;
    private static double robotPos[] = {0,0};
    public static double limelight_kp = 0.01 ;
    public static double limelight_ki = 0.05 ;
    public static double limelight_kd = 0.00 ;
    public static double odo_kp = 0.75 ;
    public static double odo_ki = 0.0 ;
    public static double odo_kd = 0.0 ;
    public static double power = 0;
    public static double kf = 0.01 ;
    public static double targetAngle = 0;
    double counter = 0;
    double TURRET_ANGLE_THRESHOLD = 1.50;
    public static boolean startShooter = true;

    private double CLOSE_DISTANCE = 80;
    private double FAR_DISTANCE = 118;
    private double CLOSE_HOOD_DISTANCE = 17;
    private double FAR_HOOD_DISTANCE = 85;
    public static double CLOSE_DISTANCE_SPEED = 1375;
    public static double MIN_SPEED = 1200;
    public static double MAX_SPEED = 1710;
    private double FAR_DISTANCE_SPEED = 1560;
    private double FAR_DISTANCE_SPEED_BLUE = 1710;
    public static double FAR_DISTANCE_SPEED_RED = 1710;
    private double targetDistance = 0;
    // =========================================================================
    // INTEGRATED DECODE CORNER POCKET GEOMETRY (40" HYPOTENUSE / 20" DEPTH OFFSET)
    // =========================================================================
    // Vector offset math: 20 * cos(45) = 14.142 inches inward on both axes

    private double RED_GOAL_X = -55.64;
    private double RED_GOAL_Y = 58.37;
    private double BLUE_GOAL_X = -55.64;
    private double BLUE_GOAL_Y = -58.37;
    private double goal_X = 0;
    private double goal_Y = 0;
    private double robotPosX = 0;
    private double robotPosY = 0;
    private double turretRobotPosX = 0;
    private double turretRobotPosY = 0;
    private double flywheelRobotPosX = 0;
    private double flywheelRobotPosY = 0;
    private double hoodRobotPosX = 0;
    private double hoodRobotPosY = 0;
    public static double HOOD_FAR_POS = 0.2;
    public static double HOOD_CLOSE_FAR_POS = 0.3;
    public static double HOOD_CLOSE_CLOSE_POS = 0.4;
    public static double MIN_POS = 0.05;
    public static double MAX_POS = 0.5;
    private double hoodPos;
    private double TURRET_MANUAL_CONTROL_THRESHOLD = 0.1;
    private double TURRET_LIMELIGHT_CONTROL_THRESHOLD = 10;
    private double robotResetPosX = 0;
    private double robotResetPosY = 0;
    private double robotResetPosXBlue = 64;
    private double robotResetPosYBlue = 64;
    private double robotResetPosXRed = 64;
    private double robotResetPosYRed = -64;
    private double robotResetPosXBlue_B = FinalAutoTrajectories.robotEndPosBlue_B.x;
    private double robotResetPosYBlue_B = FinalAutoTrajectories.robotEndPosBlue_B.y;
    private double robotResetPosXRed_B = FinalAutoTrajectories.robotEndPosRed_B.x;
    private double robotResetPosYRed_B = FinalAutoTrajectories.robotEndPosRed_B.y;
    private double robotResetPosXBlue_A = FinalAutoTrajectories.robotEndPosBlue_A.x;
    private double robotResetPosYBlue_A = FinalAutoTrajectories.robotEndPosBlue_A.y;
    private double robotResetPosXRed_A = FinalAutoTrajectories.robotEndPosRed_A.x;
    private double robotResetPosYRed_A = FinalAutoTrajectories.robotEndPosRed_A.y;
    private double robotResetPosXBlue_Y = FinalAutoTrajectories.robotEndPosBlue_Y.x;
    private double robotResetPosYBlue_Y = FinalAutoTrajectories.robotEndPosBlue_Y.y;
    private double robotResetPosXRed_Y = FinalAutoTrajectories.robotEndPosRed_Y.x;
    private double robotResetPosYRed_Y = FinalAutoTrajectories.robotEndPosRed_Y.y;
    private double robotResetAngleBlue = 90;
    private double robotResetAngleRed = 270;
    private double robotResetAngle = 0;
    private double MAX_TURRET_ANGLE = 1.4;
    private double maxTurretnAngleLimit = 1.3;

    // Configurable side-shot angular tweak baseline (in Radians)
    public static double aclTurretOffsetMin = 0.08;
    // Boundary threshold line along the X-axis to identify side-lane positions
    public static double SIDE_LANE_TRIGGER_X = -35.0;

    private double aclTurretOffsetMinBackup = 0; // Keeping structure clean
    private double flywhhelResetPosAdjustX = 0;
    private double flywhhelResetPosAdjustY = 0;
    private double flywhhelResetPosAdjustXBlue = 144;
    private double flywhhelResetPosAdjustYBlue = 144;
    private double flywhhelResetPosAdjustXRed = 0;
    private double flywhhelResetPosAdjustYRed = 0;
    private double totalEncoderAngle = 0;
    private double delta = 0;

    private double targetRelativeAngle = 0;
    private double turretBearingError = 0;
    private boolean limelightOdoReset = false;
    private double LIMELIGHT_ODO_OFFSET_X = 72;
    private double LIMELIGHT_ODO_OFFSET_Y = 72;

    // Tracking
    private double currentEncoderAngle = 0;
    private double initEncoderAngle = 0;
    private double lastEncoderAngle = 0;
    private double targetFieldAngle = 0;
    private double totalTurretAngle = 0;
    private double targetTurretAngle = 0;
    private double robotHeading = 0;
    private double robotStartHeading = 0;
    private int encoderRotations = 0;
    private double turretStartingAngle = 0;
    private double turretAngleOffset = 0;
    private double calTurretRelativeAngle = 0;
    private double calTurretRelativeAngleOffset = 0;
    private boolean initStates = false;
    private boolean turretOdoAutoAimEnabled = false;
    private double turretAbsoluteAngle = 0;
    private double turretZeroOffsetAngle = 0;

    // Constants
    private final double MAX_VOLTAGE = 3.3;
    private final double TWO_PI = 2.0 * Math.PI;
    private final double HALF_PI = 0.5 * Math.PI;

    // Use 5.812 if that is your exact physical gear ratio.
    private final double GEAR_RATIO = 5.812;

    private Pose2d AutoPose = PostStorage.currentPose;
    private double EndTurretAngle = PostStorage.endAutoTurretPos;

    private Pose3D limelightPose;

    public enum Team {
        Blue, Red
    }

    private int i = 0;
    private final Team team;


    public AutoAimTurretCommand(Shooter_Subsystem subsystem, double TargetAngle, MyRobot robot, boolean StartShooter, DoubleSupplier turretMover, Team team){
        this.robot = robot;
        this.team = team;
        scoringShooterSubsystem = subsystem;
        startShooter = StartShooter;
        targetAngle = TargetAngle;
        turretSupplier = turretMover;

        addRequirements(scoringShooterSubsystem);
    }

    @Override
    public void initialize() {
        limelight_PID = new PIDController(limelight_kp, limelight_ki, limelight_kd);
        odo_PID = new PIDController(odo_kp, odo_ki, odo_kd);

        turretAnalog = robot.hardwareMap.get(AnalogInput.class, "turretFeedback");

        odo = robot.hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setOffsets(-3.75, -3.17, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        if (team == Team.Blue) {
            robotResetPosX = robotResetPosXBlue_B;
            robotResetPosY = robotResetPosYBlue_B;
            robotResetAngle = robotResetAngleBlue;
            turretAngleOffset = PI;
        }
        if (team == Team.Red){
            robotResetAngle = robotResetAngleRed;
            robotResetPosX = robotResetPosXRed_B;
            robotResetPosY = robotResetPosYBlue_B;
            turretAngleOffset = 0;
        }

        odo.setPosition(new Pose2D(DistanceUnit.INCH, AutoPose.position.x, AutoPose.position.y, AngleUnit.RADIANS, (Math.toRadians(robotResetAngle) + AutoPose.heading.real)));
        odo.update();

        counter = 0;

        robotStartHeading =  -Math.toRadians(robotResetAngle);

//        if (team == Team.Blue) {
//            odo.setPosition(new Pose2D(DistanceUnit.INCH, robotResetPosXBlue_B, robotResetPosYBlue_B, AngleUnit.RADIANS, Math.toRadians(robotResetAngleBlue)));
//        }
//        if (team == Team.Red) {
//            odo.setPosition(new Pose2D(DistanceUnit.INCH, robotResetPosXRed_B, robotResetPosYRed_B, AngleUnit.RADIANS, Math.toRadians(robotResetAngleRed)));
//        }
//        odo.update();
        Pose2D robotPose = odo.getPosition();

        if (team == Team.Blue) {
            goal_X = BLUE_GOAL_X;
            goal_Y = BLUE_GOAL_Y;
            flywheelRobotPosX = robotPose.getX(DistanceUnit.INCH);
            flywheelRobotPosY = robotPose.getY(DistanceUnit.INCH);
            turretRobotPosX = robotPose.getX(DistanceUnit.INCH);
            turretRobotPosY = robotPose.getY(DistanceUnit.INCH);
        }
        if (team == Team.Red){
            goal_X = RED_GOAL_X;
            goal_Y = RED_GOAL_Y;
            flywheelRobotPosX = robotPose.getX(DistanceUnit.INCH);
            flywheelRobotPosY = robotPose.getY(DistanceUnit.INCH);
            turretRobotPosX = robotPose.getX(DistanceUnit.INCH);
            turretRobotPosY = robotPose.getY(DistanceUnit.INCH);
        }

        // Calculate target absolute angle based on robot position on the field
        targetFieldAngle = atan2(goal_X - turretRobotPosX , goal_Y - turretRobotPosY);
        // wrap angles
        while (targetFieldAngle > 0) targetFieldAngle -= TWO_PI;
        while (targetFieldAngle < -TWO_PI) targetFieldAngle += TWO_PI;

        if (Math.abs(targetRelativeAngle - targetFieldAngle) > PI){
            if (Math.abs(targetRelativeAngle) > PI){
                if(targetRelativeAngle > PI){
                    targetRelativeAngle -= TWO_PI;
                }else{
                    targetRelativeAngle += TWO_PI;
                }
            }else if (Math.abs(targetFieldAngle) > PI) {
                if (targetFieldAngle > PI) {
                    targetFieldAngle -= TWO_PI;
                } else {
                    targetFieldAngle += TWO_PI;
                }
            }
        }

        calTurretRelativeAngleOffset = targetFieldAngle - (-(Math.toRadians(robotResetAngle)) + (EndTurretAngle));
        turretOdoAutoAimEnabled = true;

    }

    public void execute() {
        turretProp = scoringShooterSubsystem.detectAprilTag();
        turretBearing = (turretProp[0]);
        turretRange = (turretProp[1]);

        // * * * * Odometry based flywheel speed calculation * * * *
        odo.update();
        Pose2D robotPose = odo.getPosition();

        if (team == Team.Blue) {
            goal_X = BLUE_GOAL_X;
            goal_Y = BLUE_GOAL_Y;
            flywheelRobotPosX = robotPose.getX(DistanceUnit.INCH);
            flywheelRobotPosY = robotPose.getY(DistanceUnit.INCH);
            turretRobotPosX = robotPose.getX(DistanceUnit.INCH);
            turretRobotPosY = robotPose.getY(DistanceUnit.INCH);
            hoodRobotPosX = robotPose.getX(DistanceUnit.INCH);
            hoodRobotPosY = robotPose.getY(DistanceUnit.INCH);
            FAR_DISTANCE_SPEED = FAR_DISTANCE_SPEED_BLUE;
        }
        if (team == Team.Red){
            goal_X = RED_GOAL_X;
            goal_Y = RED_GOAL_Y;
            flywheelRobotPosX = robotPose.getX(DistanceUnit.INCH);
            flywheelRobotPosY = robotPose.getY(DistanceUnit.INCH);
            turretRobotPosX = robotPose.getX(DistanceUnit.INCH);
            turretRobotPosY = robotPose.getY(DistanceUnit.INCH);
            hoodRobotPosX = robotPose.getX(DistanceUnit.INCH);
            hoodRobotPosY = robotPose.getY(DistanceUnit.INCH);
            FAR_DISTANCE_SPEED = FAR_DISTANCE_SPEED_RED;
        }

        // * * * * Hood Pos Calculation  * * * *
        // * * * * * * * * * * * * * * * * * * * * * *
        if (turretRobotPosX > 8){
            hoodPos = HOOD_FAR_POS;
            scoringShooterSubsystem.setHoodPosition(hoodPos);
        }
        else {
            targetDistance = Math.hypot(goal_X - hoodRobotPosX, goal_Y - hoodRobotPosY);

            hoodPos = HOOD_CLOSE_CLOSE_POS + (targetDistance - CLOSE_HOOD_DISTANCE) * ((HOOD_CLOSE_CLOSE_POS - HOOD_CLOSE_FAR_POS) / (FAR_HOOD_DISTANCE - CLOSE_HOOD_DISTANCE));
            hoodPos = Math.max(hoodPos, MIN_POS);
            hoodPos = Math.min(hoodPos, MAX_POS);
            scoringShooterSubsystem.setHoodPosition(hoodPos);
        }

        // * * * * Flywheel Speed Calculation  * * * *
        // * * * * * * * * * * * * * * * * * * * * * *
        targetDistance = Math.hypot(goal_X - flywheelRobotPosX, goal_Y - flywheelRobotPosY);

        shooterPower = CLOSE_DISTANCE_SPEED + (targetDistance - CLOSE_DISTANCE) * ((FAR_DISTANCE_SPEED - CLOSE_DISTANCE_SPEED) / (FAR_DISTANCE - CLOSE_DISTANCE));
        shooterPower = Math.max(shooterPower, MIN_SPEED);
        shooterPower = Math.min(shooterPower, MAX_SPEED);
        scoringShooterSubsystem.setVelocity(shooterPower);

        // * * * * Odometry Based Turret Angle Calculation * * * *
        // * * * * * * * * * * * * * * * * * * * * * * * * * * * *
        currentEncoderAngle = (turretAnalog.getVoltage() / MAX_VOLTAGE) * TWO_PI; // goes from 0 to 2 PI
        delta = currentEncoderAngle - lastEncoderAngle;
        lastEncoderAngle = currentEncoderAngle;
        // wrap angles
        if (delta < -Math.PI) {         // Wrapped forward
            encoderRotations++;
        }
        else if (delta > Math.PI){      // Wrapped backward
            encoderRotations--;
        }
        // Total angler (Radians) the SERVO has spun
        totalEncoderAngle = (encoderRotations * TWO_PI) + currentEncoderAngle;
        totalTurretAngle = (totalEncoderAngle / GEAR_RATIO);
//        targetTurretAngle = totalTurretAngle + turretAngleOffset;

        // Calculate Turret angle relative to robot heading
        robotHeading =  -robotPose.getHeading(AngleUnit.RADIANS);
//        targetRelativeAngle =  calTurretRelativeAngleOffset + Math.toRadians(robotResetAngle) + totalTurretAngle - robotHeading;
        targetRelativeAngle =  robotStartHeading + (robotHeading - robotStartHeading) + (totalTurretAngle + calTurretRelativeAngleOffset);
//        if (team == Team.Blue) {
        while (targetRelativeAngle > 0) targetRelativeAngle -= TWO_PI;
        while (targetRelativeAngle < -TWO_PI) targetRelativeAngle += TWO_PI;
//        }
//        if (team == Team.Red) {
//            while (targetRelativeAngle > TWO_PI) targetRelativeAngle -= TWO_PI;
//            while (targetRelativeAngle < 0) targetRelativeAngle += TWO_PI;
//        }

        // Calculate target absolute angle based on robot position on the field
        targetFieldAngle = atan2(goal_X - turretRobotPosX , goal_Y - turretRobotPosY);
        // wrap angles
        while (targetFieldAngle > 0) targetFieldAngle -= TWO_PI;
        while (targetFieldAngle < -TWO_PI) targetFieldAngle += TWO_PI;

        if (Math.abs(targetRelativeAngle - targetFieldAngle) > PI){
            if (Math.abs(targetRelativeAngle) > PI){
                if(targetRelativeAngle > PI){
                    targetRelativeAngle -= TWO_PI;
                }else{
                    targetRelativeAngle += TWO_PI;
                }
            }else if (Math.abs(targetFieldAngle) > PI) {
                if (targetFieldAngle > PI) {
                    targetFieldAngle -= TWO_PI;
                } else {
                    targetFieldAngle += TWO_PI;
                }
            }
        }

        turretAbsoluteAngle = targetRelativeAngle - (robotHeading - robotStartHeading) + turretAngleOffset;
        while (turretAbsoluteAngle > PI) turretAbsoluteAngle -= TWO_PI;
        while (turretAbsoluteAngle < -PI) turretAbsoluteAngle += TWO_PI;

        // * * * * Limelight based flywheel speed calculation  * * * *
        // * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
        /*if ((distance <=72.9) && (distance >=60)){
            shooterPower = 1300;
        } else if ((distance <=59.9) && (distance >=45)){
            shooterPower = 1200;
        } else if ((distance <=80.5) && (distance >=73)){
            shooterPower = 1200;
        }else {
            shooterPower = 0;
        }*/

        /*if ((Math.abs(turretRange)!= 0) && startShooter) {
            distance = (height2 - height1) / Math.tan(Math.toRadians(angle1 + turretRange));
            if (robot.operator.getButton(GamepadKeys.Button.DPAD_UP)) {
                shooterPower = higherPower + (distance - 70) * ((1700.0 - 1300.0) / (124.0 - 70.0));
            } else if (robot.operator.getButton(GamepadKeys.Button.DPAD_DOWN)){
                shooterPower = lowerPower + (distance - 70) * ((1700.0 - 1300.0) / (124.0 - 70.0));
            } else {
                shooterPower = 1300 + (distance -70) * ((1700.0 - 1300.0) / (124.0 - 70.0));
            }
        } else if (!startShooter){
            shooterPower = 10;
        }*/

        // * * * * Reset Robot position * * * *
        if ((robot.driver.getButton(GamepadKeys.Button.B))) {
            if (team == Team.Blue) {
                odo.setPosition(new Pose2D(DistanceUnit.INCH, robotResetPosXBlue_B, robotResetPosYBlue_B, AngleUnit.RADIANS, Math.toRadians(robotResetAngleBlue)));
            }
            if (team == Team.Red) {
                odo.setPosition(new Pose2D(DistanceUnit.INCH, robotResetPosXRed_B, robotResetPosYRed_B, AngleUnit.RADIANS, Math.toRadians(robotResetAngleRed)));
            }
            odo.update();
            calTurretRelativeAngleOffset = targetFieldAngle - (-(Math.toRadians(robotResetAngle)) + (totalTurretAngle));
            turretOdoAutoAimEnabled = true;

//            int calcounter = 0;
//
//            while (calcounter <= 500){
//                calcounter++;
//                if (calcounter > 0 && calcounter < 100) {
//                    scoringShooterSubsystem.lightGreen();
//                } else if (calcounter > 101 && calcounter < 200) {
//                    scoringShooterSubsystem.lightRed();
//                } else if (calcounter > 201 && calcounter < 300) {
//                    scoringShooterSubsystem.lightGreen();
//                } else if (calcounter > 301 && calcounter < 400) {
//                    scoringShooterSubsystem.lightRed();
//                } else if (calcounter > 401) {
//                    scoringShooterSubsystem.lightGreen();
//                }
//           }
        }

        if ((robot.driver.getButton(GamepadKeys.Button.A))){
            if (team == Team.Blue) {
                odo.setPosition(new Pose2D(DistanceUnit.INCH, robotResetPosXBlue_A, robotResetPosYBlue_A, AngleUnit.RADIANS, Math.toRadians(robotResetAngleBlue)));
            }
            if (team == Team.Red) {
                odo.setPosition(new Pose2D(DistanceUnit.INCH, robotResetPosXRed_A, robotResetPosYRed_A, AngleUnit.RADIANS, Math.toRadians(robotResetAngleRed)));
            }
            odo.update();
            calTurretRelativeAngleOffset = targetFieldAngle - (-(Math.toRadians(robotResetAngle)) + (totalTurretAngle));
            turretOdoAutoAimEnabled = true;
        }

        if ((robot.driver.getButton(GamepadKeys.Button.Y))){
            if (team == Team.Blue) {
                odo.setPosition(new Pose2D(DistanceUnit.INCH, robotResetPosXBlue_Y, robotResetPosYBlue_Y, AngleUnit.RADIANS, Math.toRadians(robotResetAngleBlue)));
            }
            if (team == Team.Red) {
                odo.setPosition(new Pose2D(DistanceUnit.INCH, robotResetPosXRed_Y, robotResetPosYRed_Y, AngleUnit.RADIANS, Math.toRadians(robotResetAngleRed)));
            }
            odo.update();
            calTurretRelativeAngleOffset = targetFieldAngle - (-(Math.toRadians(robotResetAngle)) + (totalTurretAngle));
            turretOdoAutoAimEnabled = true;
        }

        if ((robot.driver.getButton(GamepadKeys.Button.START)) || (robot.operator.getButton(GamepadKeys.Button.START))){
            turretOdoAutoAimEnabled = false;
        }

        // * * * * Turret Rotation PIDs  * * * *
        // * * * * * * * * * * * * * * * * * * *
        // Limelight Method Priority 2
        if ((Math.abs(turretBearing) != 0) && (Math.abs(turretSupplier.getAsDouble()) <= TURRET_MANUAL_CONTROL_THRESHOLD)) {
            power = -limelight_PID.calculate(turretBearing, targetAngle);
        }
        // Manual Method Priority 1
        else if (Math.abs(turretSupplier.getAsDouble()) > TURRET_MANUAL_CONTROL_THRESHOLD) {
            power = 0.5 * turretSupplier.getAsDouble();
        }
        // Odometry Method Priority 3
        else if (turretOdoAutoAimEnabled){
            odo_PID = new PIDController(odo_kp, odo_ki, odo_kd);

            // Calculate a conditional angular correction for side-panel shots
            double sideAngleCorrection = 0.0;
            if (turretRobotPosX < SIDE_LANE_TRIGGER_X) {
                double teamModifier = (team == Team.Blue) ? 1.0 : -1.0;
                sideAngleCorrection = aclTurretOffsetMin * teamModifier;
            }

            power = odo_PID.calculate(targetRelativeAngle, targetFieldAngle + sideAngleCorrection);
            if((turretAbsoluteAngle > 1.4) && power > 0){
                power = 0;
            }
            if((turretAbsoluteAngle < -1.4) && power < 0){
                power = 0;
            }
//            // Limit Turret rotation
//            if((targetRelativeAngle > -(Math.PI) + maxTurretnAngleLimit) && ( power != 0)){
//                power = 0;
//            }
//            if((targetRelativeAngle < -(Math.PI) - maxTurretnAngleLimit) && ( power != 0)){
//                power = 0;
//            }
        }
        else{
            power = 0;
        }
        //power = Math.max(-1, Math.min(1, power));


        scoringShooterSubsystem.setTurretPower(power);

        // * * * * Light Indicators  * * * *
        // * * * * * * * * * * * * * * * * *
        if ( (Math.abs(turretBearing) != 0) && (Math.abs(targetAngle - turretBearing) < TURRET_ANGLE_THRESHOLD)){
            counter = counter + 1;
        }
        if (counter >= 5){
            scoringShooterSubsystem.lightGreen();
        } else {
            scoringShooterSubsystem.lightRed();
        }

        if (!scoringShooterSubsystem.getDetected() || Math.abs(targetAngle - turretBearing) > TURRET_ANGLE_THRESHOLD){
            counter = 0;
        }

        // * * * * Update Telemetry  * * * *
        // * * * * * * * * * * * * * * * * *
        scoringShooterSubsystem.panelTelemetry(
                turretBearing,
                power,
                shooterPower,
                targetDistance,
                robotPose,
                targetRelativeAngle,
                currentEncoderAngle,
                delta,
                totalEncoderAngle,
                lastEncoderAngle,
                totalTurretAngle,
                targetFieldAngle,
                turretAbsoluteAngle,
                targetTurretAngle,
                robotResetAngle,
                calTurretRelativeAngleOffset,
                hoodPos,
                turretZeroOffsetAngle,
                AutoPose,
                EndTurretAngle);
    }
}