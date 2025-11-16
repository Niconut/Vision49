package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * AprilTag-based turret centering with filtering and correct control direction.
 * Uses the FTC "easy" AprilTag initialization method.
 */
@TeleOp(name = "AprilTag Manual Servo ", group = "AprilTag")
public class AprilTagMServo extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true = webcam, false = phone camera

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private CRServo mymotor;  // your servo name preserved
    private ElapsedTime timer = new ElapsedTime();

    // Persistent filter variable
    private double oldYaw = 0;

    @Override
    public void runOpMode() {

        // Initialize AprilTag vision using the easy API
        initAprilTag();

        // Initialize continuous rotation servo
        mymotor = hardwareMap.get(CRServo.class, "motor"); // your original name
        mymotor.setDirection(CRServo.Direction.REVERSE);
        telemetry.addData("Status", "Initialized — press START");
        telemetry.update();
        waitForStart();

        timer.reset();

        while (opModeIsActive()) {

            // --- Get current detections ---
            List<AprilTagDetection> detections = aprilTag.getDetections();
            telemetry.addData("# Tags Detected", detections.size());

            if (detections.isEmpty()) {
                // No tag visible → stop turret
                mymotor.setPower(0);
                telemetry.addLine("No tag detected → motor stopped");
            } else {
                // Use the first visible tag
                AprilTagDetection detection = detections.get(0);

                if (detection.metadata != null) {
                    double yaw = detection.ftcPose.yaw;  // degrees
                    double targetYaw = 0;                // centered target = 0°

                    // --- Low-pass filter parameters ---
                    double a = 10;
                    double b = 256;

                    // Proper filtering (no negative sign)
                    double yawFiltered = (a * yaw + (b - a) * oldYaw) / b;
                    oldYaw = yawFiltered;

                    // --- Proportional control ---
                    double kp = 0.01;
                    double error = targetYaw - yawFiltered;
                    double motorPower = kp * error;

                    // --- Deadband to prevent jitter ---
                    if (Math.abs(error) < 0.5) {
                        motorPower = 0;
                    }

                    // --- Safety clipping ---
                    motorPower = Math.max(-1, Math.min(1, motorPower));

                    // --- Apply power ---
                    mymotor.setPower(motorPower);

                    // --- Telemetry ---
                    telemetry.addLine(String.format("Tag ID: %d (%s)", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("Pose XYZ (in): x=%6.1f y=%6.1f z=%6.1f",
                            detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addData("Yaw (deg)", "%.2f", yaw);
                    telemetry.addData("Yaw Filtered", "%.2f", yawFiltered);
                    telemetry.addData("Error", "%.2f", error);
                    telemetry.addData("Motor Power", "%.2f", motorPower);
                } else {
                    telemetry.addLine(String.format("Unknown tag (ID %d)", detection.id));
                    telemetry.addLine(String.format("Center: (%.0f, %.0f)", detection.center.x, detection.center.y));
                }
            }

            telemetry.update();
            sleep(20);
        }

        // Stop vision safely
        visionPortal.close();
    }

    /**
     * Initialize AprilTag processor and VisionPortal the easy way.
     */
    private void initAprilTag() {
        // Create the AprilTag processor
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag); // your webcam name preserved
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }
    }
}
