package org.firstinspires.ftc.teamcode;// Import required Limelight library classes
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.CRServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// Basic OpMode for reading Limelight data
@TeleOp(name = "Limelight distance main")
public class Limelightdistance2 extends LinearOpMode {
    private DcMotorEx launcher;
    private DcMotorEx intake1;
    private DcMotorEx intake2;
    private DcMotorEx intake3;



    @Override
    public void runOpMode() {
        // Initialize Limelight from hardware map
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        launcher = hardwareMap.get(DcMotorEx.class, "ScoringShooter");
        intake1 = hardwareMap.get(DcMotorEx.class,"frontIntake");
        intake2 = hardwareMap.get(DcMotorEx.class,"midIntake");
        intake3 = hardwareMap.get(DcMotorEx.class,"backIntake");
        launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Status", "Initialized — press START");
        telemetry.update();



        // Start streaming and polling data
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            intake1.setPower(-0.5);
            intake2.setPower(-0.5);
            intake3.setPower(0.5);
            // Get the latest result container
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                // ty: Vertical offset (degrees)
                // ta: Target area (0%-100% of image)
                double height1 = 16;// height of limelight lens from the floor
                double ty = result.getTy();// vertical offset
                double height2 = 29.5;// height of center of apriltag to the floor
                double angle1 = 0;// mounting angle of the limelight degrees back from vertical
                double distance = (height2 - height1)/ Math.tan(Math.toRadians(angle1 + ty));
                double power = 1300 + (distance - 70) * ( (1625.0 - 1300.0) / (124.0 - 70.0) );
                launcher.setVelocity(power);









                telemetry.addData("power", power);
                telemetry.addData("ty", ty);
                telemetry.addData("distance", distance);
            } else {
                telemetry.addData("Status", "No Target Detected");
            }
            telemetry.update();
        }
    }
}
