package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name="SpeedControl_Matched_Coords", group="Pinpoint")
public class Odometryspeedcontrolright extends LinearOpMode {

    private GoBildaPinpointDriver odo;
    private DcMotorEx launcher;

    private final double GOAL_X = -58.37;
    private final double GOAL_Y = 55.64;

    @Override
    public void runOpMode() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        launcher = hardwareMap.get(DcMotorEx.class, "ScoringShooter");

        // Pinpoint Configuration
        odo.setOffsets(-3.75, -3.17, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        waitForStart();

        while (opModeIsActive()) {
            odo.update();


            Pose2D robotPose = odo.getPosition();
            double currentX = robotPose.getX(DistanceUnit.INCH);
            double currentY = robotPose.getY(DistanceUnit.INCH);


            double dx = GOAL_X - currentX;
            double dy = GOAL_Y - currentY;


            double floorDistance = Math.hypot(dx, dy);


            double power = 975 + (floorDistance - 70) * ((1300.0 - 975.0) / (124.0 - 70.0));


            power = Math.max(0, Math.min(2500, power));
            launcher.setVelocity(power);


            telemetry.addData("Dist to Goal", "%.2f in", floorDistance);
            telemetry.addData("Launcher Velocity", "%.1f", power);
            telemetry.addData("Robot Pose", "X: %.1f, Y: %.1f", currentX, currentY);
            telemetry.update();
        }
    }
}

