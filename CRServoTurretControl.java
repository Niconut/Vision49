package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// Changed from DcMotorEx to CRServo
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;

@TeleOp(name="Turret CRServo Profiled Tracking")
public class CRServoTurretControl extends LinearOpMode {


    private CRServo turretServo;
    private Limelight3A limelight;

    // Profiled PID setup
    private final double Kp = 0.1;
    private final double Ki = 0.0;
    private final double Kd = 0.000;

    // Constraints: (Max Velocity deg/s, Max Acceleration deg/s^2)
    private final TrapezoidProfile.Constraints constraints =
            new TrapezoidProfile.Constraints(300, 150);

    private final ProfiledPIDController controller =
            new ProfiledPIDController(Kp, Ki, Kd, constraints);

    @Override
    public void runOpMode() {

        turretServo = hardwareMap.get(CRServo.class, "ScoringTurret");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Initialization
        limelight.pipelineSwitch(0);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result.isValid()) {
                double tx = result.getTx();


                // Calculate smooth power
                double power = (controller.calculate(tx,0.0));
                if(abs(tx)<9){
                    power = (power/4);
                }

                // Apply power to the CRServo
                turretServo.setPower(-(power));


                telemetry.addData("Status", "Tracking");
                telemetry.addData("Error (tx)", tx);
                telemetry.addData("Servo Power", power);

            } else {
                // No target found, stop servo
                turretServo.setPower(0);
                telemetry.addData("Status", "No Target Found");
            }
            telemetry.update();
        }
    }
}
