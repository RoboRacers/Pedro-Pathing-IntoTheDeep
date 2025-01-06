package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Half Arm Control with PIDF Dashboard", group = "Arm")
@Config // Enables configuration via FTC Dashboard
public class PitchPIDHalf extends LinearOpMode {

    private DcMotor armMotor;
    private AnalogInput potentiometer;
    private FtcDashboard dashboard;

    double motorPower;
    double maxPower;

    // Configuration variables (tunable via dashboard)
    public static double kP = 0.042;
    public static double kI = 0.001;
    public static double kD = 0.001;
    public static double kF = 0.4;
    public static double targetAngle = 0.0; // Target angle in degrees

    private double integralSum = 0;
    private double lastError = 0;
    private double lastTarget = 0;

    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        armMotor = hardwareMap.get(DcMotor.class, "pitchMotor");
        potentiometer = hardwareMap.get(AnalogInput.class, "pot");

        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize FTC Dashboard
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {

            double currentVoltage = potentiometer.getVoltage();
            double currentAngle = mapPotentiometerToAngle(currentVoltage);

            // Calculate error (using angles)
            double error = targetAngle - currentAngle;
            if (targetAngle>lastTarget){
//            if (targetAngle>lastTarget){
                kP = 0.05;
                kD = 0.0055;
                kI = 0.00001;
                kF = 0.4;
            }else if (targetAngle<lastTarget){
//            }else if (targetAngle<currentAngle){
                kP = 0.0055;
                kD = 0.000065;
                kI = 0.0001;
                kF= 0.1;
            }

            if(Math.abs(error) < 10 && Math.abs(error)>1){
                lastTarget = targetAngle;
//                kP = 0.054;
//                kD = 0.0015;
                kI = 0.004;
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
//            lastTarget = targetAngle;   //useless because as soon as the pid values are set the last target becomes the target value we set meaning the pid values will not adjust to whether the pitch goes up or down
            timer.reset();

            // Telemetry to dashboard
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Target LAst", lastTarget);
            telemetry.addData("Current Angle", currentAngle);
            telemetry.addData("Error", error);
            telemetry.addData("Last error", lastError);
            telemetry.addData("Motor Power", motorPower);
            telemetry.addData("kP", kP);
            telemetry.addData("kI", kI);
            telemetry.addData("kD", kD);
            telemetry.addData("kF", kF);
            telemetry.addData("Max Power Used", maxPower);
            telemetry.addData("Pot Voltage", potentiometer.getVoltage());
            telemetry.update();
            telemetry.update();// Important: Update the dashboard
            dashboard.getTelemetry();
        }
    }

    private double mapPotentiometerToAngle(double potentiometerValue) {

//        return ((oldval - oldmin)/ (oldmax-oldmin)) * (newmax - newmin) +newmin

//        return 0 + ((90 - 0) / (0.64 - 0.0005)) * (potentiometerValue - 0);
        return ((potentiometerValue - 0.47800000000000004)/ (1.1360000000000001-0.47800000000000004)) * (90 - 0) -0;

    }
}