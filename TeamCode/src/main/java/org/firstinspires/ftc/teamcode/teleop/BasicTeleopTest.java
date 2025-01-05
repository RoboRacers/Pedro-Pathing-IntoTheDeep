package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.Assembly;

@TeleOp(name = "Basic Teleop Test", group = "Test")
public class BasicTeleopTest extends LinearOpMode {

    private Assembly assembly;
    Servo claw;
    DcMotorImplEx pitchMotor;
    DcMotorImplEx slidesMotor;
    private FtcDashboard dashboard;

    public static double targetValue = 10;

    @Override
    public void runOpMode() throws InterruptedException {
        while(opModeInInit()){
            assembly = new Assembly(hardwareMap);
        }

        waitForStart();

        while(!isStopRequested()) {
            if (gamepad1.dpad_up){
                targetValue = 40;
            }else if(gamepad1.dpad_right){
                targetValue = 20;
            }else if(gamepad1.dpad_left){
                targetValue = 10;
            }

            assembly.extendSlide(targetValue);
            telemetry.addData("Target", targetValue);


            telemetry.update();
        }
    }
}