package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "simple slid PIDF", group = "Arm")
@Config // Enables configuration via FTC Dashboard
public class SlidesTest extends LinearOpMode {

    private DcMotor slidesMotor;
    private Rev2mDistanceSensor distanceSensor;
    private AnalogInput pot;

    public WeightedMovingAverage wma = new WeightedMovingAverage(0.7);
    private FtcDashboard dashboard;

    double motorPower;
    double maxPower;

    // Configuration variables (tunable via dashboard)
    public static double kP = 0.042;
    public static double kI = 0.001;
    public static double kD = 0.002;
    public static double kF = 0.39;
    public static double targetLength = 0.0; // Target angle in degrees

    private double integralSum = 0;
    private double lastError = 0;
    private double lastTarget = 0;

    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        slidesMotor = hardwareMap.get(DcMotor.class, "slidesMotor");
        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distance");

        slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slidesMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        pot = hardwareMap.get(AnalogInput.class,"pot");
        // Initialize FTC Dashboard
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            double curPosRaw = distanceSensor.getDistance(DistanceUnit.CM);
            double actualCurPos = rawToGoodWithFilter(curPosRaw);

            // Calculate error (using angles)
            double error = targetLength - actualCurPos;

            integralSum += error * timer.seconds();

            double derivative = (error - lastError) / timer.seconds();

            double feedForward = kF * Math.sin(Math.toRadians(mapPotentiometerToAngle(pot.getVoltage())));

            motorPower = (kP * error) + (kI * integralSum) + (kD * derivative) + feedForward;

            motorPower = Math.max(-1.0, Math.min(1.0, motorPower));

            if(Math.abs(motorPower) > Math.abs(maxPower)){
                maxPower = motorPower;
            }

            slidesMotor.setPower(motorPower);

            lastError = error;
            lastTarget = targetLength;
            timer.reset();

            // Telemetry to dashboard
            telemetry.addData("Target Pos", targetLength);
            telemetry.addData("Current Pos Actual", actualCurPos);
            telemetry.addData("Current Pos Raw", curPosRaw);
            telemetry.addData("Error", error);
            telemetry.addData("Motor Power", motorPower);
            telemetry.addData("kP", kP);
            telemetry.addData("kI", kI);
            telemetry.addData("kD", kD);
            telemetry.addData("kF", kF);
            telemetry.addData("Max Power Used", maxPower);
            telemetry.update();
            telemetry.update();// Important: Update the dashboard
            dashboard.getTelemetry();
        }
    }

    private double rawToGoodWithFilter(double pos) {
        double actualPos;
        if(pos>6) {
            actualPos = 0.0000330052 * (Math.pow(pos, 4)) -0.00356719 * (Math.pow(pos, 3)) + 0.137523 * (Math.pow(pos, 2))  -1.15156*pos + 9.04499;
        }
        else{
            actualPos = pos;
        }
        return Math.round(actualPos);
    }

    private double mapPotentiometerToAngle(double potentiometerValue) {
        return ((potentiometerValue - 0.47800000000000004)/ (1.1360000000000001-0.47800000000000004)) * (90 - 0) -0;
    }
}