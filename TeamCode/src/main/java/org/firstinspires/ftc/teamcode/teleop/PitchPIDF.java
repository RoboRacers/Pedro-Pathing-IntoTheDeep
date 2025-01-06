package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Arm Control with PIDF Dashboard", group = "Arm")
@Config // Enables configuration via FTC Dashboard
public class PitchPIDF extends LinearOpMode {

    private DcMotor armMotor;
    private AnalogInput pot; //potencameter
    private FtcDashboard dashboard;

    double motorPower;
    double maxPower;

    // Configuration variables (tunable via dashboard)
    public static double kP = 0.042;
    public static double kI = 0.001;
    public static double kD = 0.001;
    public static double kF = 0.4;

    public static double kPup = 0.026;
    public static double kIup = 0.000;
    public static double kDup = 0.003;
    public static double kFup = 0.22;

    public static double kPdown = 0.009;
    public static double kIdown = 0.00;
    public static double kDdown = 0.00;
    //public static double kDdown = 0.000005;
    public static double kFdown = 0.1;

    public static double kIerror = 0.0018;

    public static double targetAngle = 0.0; // Target angle in degrees

    private double integralSum = 0;
    private double lastError = 0;
    private double lastTarget = 0;

    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        armMotor = hardwareMap.get(DcMotor.class, "pitchMotor");
        pot = hardwareMap.get(AnalogInput.class, "pot");

        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize FTC Dashboard
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            double currentVoltage = pot.getVoltage();
            double currentAngle = mapPotentiometerToAngle(currentVoltage);

            // Calculate error (using angles)
            double error = targetAngle - currentAngle;

            if (targetAngle>lastTarget){
                kP = kPup;
                kD = kDup;
                kI = kIup;
                kF = kFup;
            }else if (targetAngle<lastTarget){
                kP = kPdown;
                kD = kDdown;
                kI = kIdown;
                kF= kFdown;
            }

            if(Math.abs(error) < 10 && Math.abs(error)>1){
//                kP = 0.054;
//                kD = 0.0015;
                kI = kIerror;
            }

            integralSum += error * timer.seconds();


            double derivative = (error - lastError) / timer.seconds();

            double feedForward = kF * Math.cos(Math.toRadians(currentAngle));

            motorPower = (kP * error) + (kI * integralSum) + (kD * derivative) + feedForward;

            motorPower = Math.max(-1.0, Math.min(1.0, motorPower));

            if(Math.abs(motorPower) > Math.abs(maxPower)){
                maxPower = motorPower;
            }

            armMotor.setPower(motorPower);

            lastError = error;
            lastTarget = targetAngle;
            timer.reset();

            // Telemetry to dashboard
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Current Angle", currentAngle);
            telemetry.addData("Error", error);
            telemetry.addData("Motor Power", motorPower);
            telemetry.addData("kP", kP);
            telemetry.addData("kI", kI);
            telemetry.addData("kD", kD);
            telemetry.addData("kF", kF);
            telemetry.addData("Max Power Used", maxPower);
            telemetry.addData("Pot Voltage", pot.getVoltage());
            telemetry.update();
            telemetry.update();// Important: Update the dashboard
            dashboard.getTelemetry();
        }
    }

    private double mapPotentiometerToAngle(double potentiometerValue) {

        return ((potentiometerValue - 0.47800000000000004)/ (1.1360000000000001-0.47800000000000004)) * (90 - 0) -0;
    }
}