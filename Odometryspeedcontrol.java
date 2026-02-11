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

@TeleOp(name="Analog Turret Logic", group="Pinpoint")
public class Odometryspeedcontrol extends LinearOpMode {

    // Hardware
    private CRServoImplEx turretServo;
    private AnalogInput turretAnalog; // Only source of position data
    private GoBildaPinpointDriver odo;
    private DcMotorEx launcher;


    @Override
    public void runOpMode() {

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        launcher = hardwareMap.get(DcMotorEx.class, "ScoringShooter");


        odo.setOffsets(-3.75, -3.17, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();


        waitForStart();

        while (opModeIsActive()) {
            odo.update();



            Pose2D robotPose = odo.getPosition();
            double y = robotPose.getX(DistanceUnit.INCH);
            double x = robotPose.getY(DistanceUnit.INCH);
            double H = robotPose.getHeading(AngleUnit.RADIANS);
            double robotH = -(H);
            double RED_GOAL_X = 13.63;
            double RED_GOAL_Y = 127.64;

            double dx = RED_GOAL_X - x;
            double dy = RED_GOAL_Y - y;
            double floorDistance = Math.hypot(dx, dy);


            double power = 975 + (floorDistance - 70) * ((1300.0 - 975.0) / (124.0 - 70.0));

            launcher.setVelocity(power);


            telemetry.addData("pos y", x);
            telemetry.addData("pos x", y);
            telemetry.addData("heading", robotH);
            telemetry.addData("velocity", power);
            telemetry.update();
        }
    }
}


