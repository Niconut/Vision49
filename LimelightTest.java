package org.firstinspires.ftc.teamcode;// Import required Limelight library classes
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// Basic OpMode for reading Limelight data
@TeleOp(name = "Limelight Basic Test")
public class LimelightTest extends LinearOpMode {
    private CRServo turret;


    @Override
    public void runOpMode() {
        // Initialize Limelight from hardware map
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turret = hardwareMap.get(CRServo.class, "turret");
        turret.setDirection(CRServo.Direction.REVERSE);
        telemetry.addData("Status", "Initialized — press START");
        telemetry.update();



        // Start streaming and polling data
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            // Get the latest result container
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                // tx: Horizontal offset (degrees)
                // ty: Vertical offset (degrees)
                // ta: Target area (0%-100% of image)
                double tx = result.getTx();
                double ty = result.getTy();
                double ta = result.getTa();
                double targettx = 0;
                double kp = 0.03 ;
                double error = tx-targettx;
                double motorpower = kp*error;


                // --- Deadband to prevent jitter ---
                if (Math.abs(error) < 0.5) {
                    motorpower = 0;
                }

                // --- Safety clipping ---
                motorpower = Math.max(-1, Math.min(1, motorpower));
                turret.setPower(motorpower);

                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);
                telemetry.addData("Tx", tx);
                telemetry.addData("MotorPower", motorpower);
            } else {
                telemetry.addData("Status", "No Target Detected");
            }
            telemetry.update();
        }
    }
}
