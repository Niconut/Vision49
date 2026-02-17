package org.firstinspires.ftc.teamcode;

import static java.lang.Math.atan2;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

@TeleOp(name="Turret_Final_ShortestPath", group="Control")
public class Turretcontrol extends LinearOpMode {

    // Hardware
    private CRServoImplEx turretServo;
    private AnalogInput turretAnalog;
    private GoBildaPinpointDriver odo;

    // Tracking
    private double lastEncoderRad = 0;
    private int encoderRotations = 0;
    private double totalTurretRadians = 0;

    // Constants
    private final double MAX_VOLTAGE = 5.0;
    private final double TWO_PI = 2.0 * Math.PI;

    // ADJUST THIS: 5.0 if 5 servo turns = 1 turret turn.
    // Use 5.812 if that is your exact physical gear ratio.
    private final double GEAR_RATIO = 5.812;

    @Override
    public void runOpMode() {
        // 1. Hardware Map
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        turretServo = hardwareMap.get(CRServoImplEx.class, "turretServo");
        turretAnalog = hardwareMap.get(AnalogInput.class, "turretFeedback");
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");


        // 2. Odo Config
        odo.setOffsets(-3.75, -3.17, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        // Initialize starting position (Negative sign flips direction if needed)
        lastEncoderRad = (turretAnalog.getVoltage() / MAX_VOLTAGE) * TWO_PI;
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {


            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                // tx: Horizontal offset (degrees)
                // ty: Vertical offset (degrees)
                // ta: Target area (0%-100% of image)
                double tx = result.getTx();
                double ty = result.getTy();
                double ta = result.getTa();
                double targettx = 0;
                double kp = 0.03;
                double error = tx - targettx;
                double motorpower = kp * error;


                // --- Deadband to prevent jitter ---
                if (Math.abs(error) < 0.5) {
                    motorpower = 0;
                }

                // --- Safety clipping ---
                motorpower = Math.max(-1, Math.min(1, motorpower));
                turretServo.setPower(motorpower);

                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);
                telemetry.addData("Tx", tx);
                telemetry.addData("MotorPower", motorpower);
            } else {
                odo.update();

                // --- STEP 1: TRACK SERVO ROTATIONS ---
                // Read raw analog and convert to 0 - 2PI
                double currentEncoderRad = -((turretAnalog.getVoltage() / MAX_VOLTAGE) * TWO_PI);

                // Detect when the small gear (servo) passes the 5V -> 0V gap
                double delta = currentEncoderRad - lastEncoderRad;
                if (delta < -Math.PI) encoderRotations++;      // Wrapped forward
                else if (delta > Math.PI) encoderRotations--; // Wrapped backward

                // Total radians the SERVO has spun
                double totalEncoderRad = (encoderRotations * TWO_PI) + currentEncoderRad;
                lastEncoderRad = currentEncoderRad;

                // --- STEP 2: SCALE TO TURRET RADIANS ---
                // Divide by the gear ratio to get the big circle's position
                totalTurretRadians = totalEncoderRad / GEAR_RATIO;

                // --- STEP 3: CALCULATE TARGET ---
                Pose2D robotPose = odo.getPosition();
                double robotH = robotPose.getHeading(AngleUnit.RADIANS);

                // Absolute angle to Goal (Replace coordinates with your goal location)
                double targetFieldAngle = atan2(127.64 - robotPose.getX(DistanceUnit.INCH), 13.63 - robotPose.getY(DistanceUnit.INCH));

                // Where the turret should be relative to the robot's front
                double targetRelativeAngle = targetFieldAngle - robotH;

                // --- STEP 4: SHORTEST PATH LOGIC ---
                // Calculate the difference between target and current position
                double error = targetRelativeAngle - totalTurretRadians;

                // Normalize the error to the range [-PI, PI]
                // This ensures it never turns more than 180 degrees to find the target
                while (error > Math.PI) error -= TWO_PI;
                while (error < -Math.PI) error += TWO_PI;

                // --- STEP 5: OUTPUT TO SERVO ---
                double kP = 0.35; // Power multiplier
                double deadband = 0.01; // Stops jitter
                double pos = 0;

                if (Math.abs(error) > deadband) {
                    // Proportional Power + Friction Feedforward (0.04)
                    pos = -(error * kP) + (Math.signum(error) * 0.04);
                }

                // Safety Cap
                turretServo.setPower(Math.max(-1, Math.min(1, pos)));

                // --- TELEMETRY ---
                telemetry.addData("Turret Deg", Math.toDegrees(totalTurretRadians));
                telemetry.addData("Target Deg", Math.toDegrees(targetRelativeAngle));
                telemetry.addData("Error Deg", Math.toDegrees(error));
                telemetry.addData("Servo Turns", encoderRotations);
                telemetry.update();
            }
        }
    }
}
