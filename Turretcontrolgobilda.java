package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name="Turret_DECODE_Pinpoint", group="Control")
public class Turretcontrolgobilda extends LinearOpMode {

    private CRServoImplEx turretServo;
    private AnalogInput turretAnalog;
    private GoBildaPinpointDriver odo;

    private double lastEncoderRad = 0;
    private int encoderRotations = 0;
    private final double MAX_VOLTAGE = 5.0, TWO_PI = 2.0 * Math.PI, GEAR_RATIO = 5.812;

    // --- OFFICIAL DECODE COORDINATES ---
    private final double TARGET_X = -58.37; // Back of field
    private final double TARGET_Y = 55.64;  // Red Goal (Use -55.64 for Blue)

    @Override
    public void runOpMode() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        turretServo = hardwareMap.get(CRServoImplEx.class, "turretServo");
        turretAnalog = hardwareMap.get(AnalogInput.class, "turretFeedback");

        // Pinpoint initialization
        odo.setOffsets(-3.75, -3.17, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.resetPosAndIMU();

        lastEncoderRad = (turretAnalog.getVoltage() / MAX_VOLTAGE) * TWO_PI;

        waitForStart();

        while (opModeIsActive()) {
            odo.update(); // Update odometry position

            // 1. Calculate Turret Absolute Rotation
            double currentEncoderRad = -((turretAnalog.getVoltage() / MAX_VOLTAGE) * TWO_PI);
            double delta = currentEncoderRad - lastEncoderRad;
            if (delta < -Math.PI) encoderRotations++;
            else if (delta > Math.PI) encoderRotations--;

            lastEncoderRad = currentEncoderRad;
            double totalTurretRadians = ((encoderRotations * TWO_PI) + currentEncoderRad) / GEAR_RATIO;

            // 2. Field Targeting Logic
            Pose2D pose = odo.getPosition();
            // Calculate angle from robot to goal: atan2(deltaY, deltaX)
            double targetFieldAngle = Math.atan2(TARGET_Y - pose.getY(DistanceUnit.INCH),
                    TARGET_X - pose.getX(DistanceUnit.INCH));

            // 3. Shortest Path Error
            // Relative angle = (Field angle to goal) - (Robot's world heading)
            double targetRelativeAngle = targetFieldAngle - pose.getHeading(AngleUnit.RADIANS);
            double error = targetRelativeAngle - totalTurretRadians;

            // Wrap error to [-PI, PI] for shortest path turn
            while (error > Math.PI) error -= TWO_PI;
            while (error < -Math.PI) error += TWO_PI;

            // 4. Motor Output
            double power = (Math.abs(error) > 0.01) ? -(error * 0.45) + (Math.signum(error) * 0.05) : 0;
            turretServo.setPower(Math.max(-1, Math.min(1, power)));

            telemetry.addData("Robot Pos", "X: %.1f, Y: %.1f", pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH));
            telemetry.addData("Turret Angle Deg", Math.toDegrees(totalTurretRadians));
            telemetry.update();
        }
    }
}

