package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name="Turret_Smart_Limits", group="Control")
public class Turretcontrolgobildalimit extends LinearOpMode {

    private CRServoImplEx turretServo;
    private AnalogInput turretAnalog;
    private GoBildaPinpointDriver odo;

    private double lastEncoderRad = 0;
    private int encoderRotations = 0;
    private final double MAX_VOLTAGE = 5.0, TWO_PI = 2.0 * Math.PI, GEAR_RATIO = 5.812;

    // --- LIMITS ---
    private final double THIRTY_DEG_RAD = Math.toRadians(30);
    private double minLimitRad, maxLimitRad;

    // --- TARGETS ---
    private final double TARGET_X = -58.37;
    private final double TARGET_Y = 55.64;

    @Override
    public void runOpMode() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        turretServo = hardwareMap.get(CRServoImplEx.class, "turretServo");
        turretAnalog = hardwareMap.get(AnalogInput.class, "turretFeedback");

        odo.setOffsets(-3.75, -3.17, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.resetPosAndIMU();

        // 1. Determine where we are and where the goal is BEFORE the match starts
        odo.update();
        Pose2D startPose = odo.getPosition();
        double initialTargetFieldAngle = Math.atan2(TARGET_Y - startPose.getY(DistanceUnit.INCH),
                TARGET_X - startPose.getX(DistanceUnit.INCH));

        // 2. Define the "Safe Zone" based on that initial heading
        double initialRelativeTarget = initialTargetFieldAngle - startPose.getHeading(AngleUnit.RADIANS);
        minLimitRad = initialRelativeTarget - THIRTY_DEG_RAD;
        maxLimitRad = initialRelativeTarget + THIRTY_DEG_RAD;

        lastEncoderRad = -(turretAnalog.getVoltage() / MAX_VOLTAGE) * TWO_PI;

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Turret limits set relative to initial goal position.");
            telemetry.addData("Allowed Range (Deg)", "%.1f to %.1f",
                    Math.toDegrees(minLimitRad), Math.toDegrees(maxLimitRad));
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            odo.update();

            // 3. Calculate Current Turret Position
            double currentEncoderRad = -((turretAnalog.getVoltage() / MAX_VOLTAGE) * TWO_PI);
            double delta = currentEncoderRad - lastEncoderRad;
            if (delta < -Math.PI) encoderRotations++;
            else if (delta > Math.PI) encoderRotations--;
            lastEncoderRad = currentEncoderRad;

            double totalTurretRadians = ((encoderRotations * TWO_PI) + currentEncoderRad) / GEAR_RATIO;

            // 4. Field Targeting
            Pose2D pose = odo.getPosition();
            double targetFieldAngle = Math.atan2(TARGET_Y - pose.getY(DistanceUnit.INCH),
                    TARGET_X - pose.getX(DistanceUnit.INCH));

            // 5. Apply "Smart" Limits
            double desiredRelativeAngle = targetFieldAngle - pose.getHeading(AngleUnit.RADIANS);

            // This forces the turret to stay within +/- 30 degrees of where it pointed at Init
            double clippedTargetAngle = Range.clip(desiredRelativeAngle, minLimitRad, maxLimitRad);

            // 6. Error and Power
            double error = clippedTargetAngle - totalTurretRadians;
            while (error > Math.PI) error -= TWO_PI;
            while (error < -Math.PI) error += TWO_PI;

            double power = (Math.abs(error) > 0.01) ? -(error * 0.45) + (Math.signum(error) * 0.05) : 0;
            turretServo.setPower(Range.clip(power, -1, 1));

            telemetry.addData("Status", (desiredRelativeAngle != clippedTargetAngle) ? "LIMITED" : "TRACKING");
            telemetry.addData("Turret Deg", Math.toDegrees(totalTurretRadians));
            telemetry.update();
        }
    }
}
