package org.firstinspires.ftc.teamcode;

import static java.lang.Math.atan2;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name="Turretcontrol", group="Pinpoint")
public class Turretcontrol extends LinearOpMode {

    // Hardware
    private CRServoImplEx turretServo;
    private AnalogInput turretAnalog; // Only source of position data
    private GoBildaPinpointDriver odo;
    private DcMotorEx launcher;

    // Tracking Variables (Must stay outside the loop)
    private double lastLocalRad = 0;
    private int fullRotations = 0;
    private double totalTurretRadians = 0;

    // Constants
    private final double MAX_VOLTAGE = 5.0;
    private final double TWO_PI = 2.0 * Math.PI;
    private final double TARGET_TOTAL_RAD = 5.0 * TWO_PI; // Max range (10*PI)

    @Override
    public void runOpMode() {
        // 1. Hardware Map
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        turretServo = hardwareMap.get(CRServoImplEx.class, "turretServo");
        turretAnalog = hardwareMap.get(AnalogInput.class, "turretFeedback");

        // 2. Pinpoint Config (Offsets in MM as per goBILDA standard)
        odo.setOffsets(-3.75, -3.17, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        // Initialize starting position based ONLY on voltage
        lastLocalRad = (turretAnalog.getVoltage() / MAX_VOLTAGE) * TWO_PI;

        waitForStart();

        while (opModeIsActive()) {
            odo.update();

            // --- STEP 1: READ ANALOG VOLTAGE (NO SERVO POSITION USED) ---
            double currentLocalRad = (turretAnalog.getVoltage() / MAX_VOLTAGE) * TWO_PI;

            // Detect rollover (crossing the 0/360 boundary)
            double delta = currentLocalRad - lastLocalRad;
            if (delta < -Math.PI) fullRotations++;      // Clockwise jump
            else if (delta > Math.PI) fullRotations--; // Counter-clockwise jump

            totalTurretRadians = (fullRotations * TWO_PI) + currentLocalRad;
            lastLocalRad = currentLocalRad;

            // --- STEP 2: CALCULATE FIELD TARGET ---7

            Pose2D robotPose = odo.getPosition();
            double y = robotPose.getX(DistanceUnit.INCH);
            double x = robotPose.getY(DistanceUnit.INCH);
            double H = robotPose.getHeading(AngleUnit.RADIANS);
            double robotH = -(H);




            // Absolute angle from robot to goal redgoal
           double targetFieldAngle = atan2(127.64 - y, 13.63- x);

            // Relative angle target (where the turret should be compared to the robot front)
            double targetRelativeAngle = targetFieldAngle - robotH;

            // --- STEP 3: DRIVE SERVO VIA POWER ---
            // Calculate error between target and our accumulated analog position
            double error = targetRelativeAngle - totalTurretRadians;

            // Normalize error to the closest rotation so it doesn't spin 5 times to fix 1 degree
            while (error > Math.PI) error -= TWO_PI;
            while (error < -Math.PI) error += TWO_PI;

            // Proportional Gain (Adjust kP to change speed/accuracy)
            double kP = 0.8;
            double pos = error * kP;

            // Safety Clamping
            pos = Math.max(-1.0, Math.min(1.0, pos));

            // Apply power to the CR Servo
            turretServo.setPower(pos);

            // --- TELEMETRY ---
            telemetry.addData("Voltage", turretAnalog.getVoltage());
            telemetry.addData("0.0-1.0 Progress", totalTurretRadians / TARGET_TOTAL_RAD);
            telemetry.addData("Turret Angle (Rad)", totalTurretRadians);
            telemetry.addData("Error", error);
            telemetry.addData("pos y", x);
            telemetry.addData("pos x", y);
            telemetry.addData("heading", robotH);
            telemetry.update();
        }
    }
}


