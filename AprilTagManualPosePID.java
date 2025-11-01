package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "AprilTag Manual Pose (Fixed PID)", group = "AprilTag")
public class AprilTagManualPosePID extends LinearOpMode {

    private static final double CUSTOM_TAG_SIZE_INCHES = 4.5;
    private static final int CUSTOM_TAG_ID = 20;

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private DcMotorEx mymotor;
    private ElapsedTime timer = new ElapsedTime();

    // --- PID Constants ---
    private double Kp = 1.0;
    private double Ki = 0.0;
    private double Kd = 0.0;

    // --- PID Variables ---
    private double integralSum = 0;
    private double lastError = 0;

    private double calculatePID(double targetPosition, double currentPosition) {
        double error = targetPosition- currentPosition;
        double dt = timer.seconds();
        timer.reset();

        double proportional = Kp * error;
        integralSum += error * dt;
        double integral = Ki * integralSum;
        double derivative = (dt > 0) ? Kd * (error - lastError) / dt : 0;

        lastError = error;
        return proportional + integral + derivative;
    }

    @Override
    public void runOpMode() {
        // 1. Define a custom AprilTag library
        AprilTagLibrary customAprilTagLibrary = new AprilTagLibrary.Builder()
                .addTag(CUSTOM_TAG_ID, "CustomTag", CUSTOM_TAG_SIZE_INCHES, DistanceUnit.INCH)
                .addTag(24, "Custom2tag", CUSTOM_TAG_SIZE_INCHES, DistanceUnit.INCH)
                .build();

        // 2. Create AprilTag processor
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(customAprilTagLibrary)
                .build();

        // 3. Set up vision portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .build();

        // 4. Configure motor
        mymotor = hardwareMap.get(DcMotorEx.class, "motor");
        mymotor.setPower(0);
        mymotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mymotor.setDirection(DcMotorSimple.Direction.FORWARD);
        mymotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        mymotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            if (currentDetections.size() == 0) {
                // No tag visible → stop and reset PID
                mymotor.setPower(0);
                mymotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                integralSum = 0;
                lastError = 0;
                telemetry.addLine("No tag detected → motor stopped");
            } else {
                // Process first visible tag (you can choose which one to prioritize)
                AprilTagDetection detection = currentDetections.get(0);

                if (detection.metadata != null) {
                    telemetry.addLine(String.format("Tag ID: %d (%s)", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("Pose XYZ (in.): x=%6.1f y=%6.1f z=%6.1f",
                            detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));

                    double yaw = detection.ftcPose.yaw;  // rotation error (degrees)
                    double targetYaw = 0; // goal = centered tag

                    // PID control for yaw
                    double motorPower = calculatePID(targetYaw, yaw);

                    // Add a deadband
                    if (Math.abs(yaw) < 5.0) {
                        motorPower = 0;
                    }

                    // Clip motor power for safety
                    motorPower = Math.max(-0.25, Math.min(0.25, motorPower));

                    mymotor.setPower(motorPower);

                    telemetry.addData("Yaw", "%.2f", yaw);
                    telemetry.addData("Motor Power", "%.2f", motorPower);
                }
            }

            telemetry.update();
            sleep(20);
        }

        // Stop camera safely
        visionPortal.close();
    }
}
