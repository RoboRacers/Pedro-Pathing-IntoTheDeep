package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.Assembly;

@TeleOp(name = "Basic Teleop Test", group = "Test")
public class BasicTeleopTest extends LinearOpMode {

    private Assembly assembly;
    Servo claw;

    @Override
    public void runOpMode() throws InterruptedException {
        while(opModeInInit()){
            assembly = new Assembly(hardwareMap);
        }

        waitForStart();

        while(!isStopRequested()) {
            //claw test here
            assembly.flipClaw(gamepad1.left_stick_y);
            telemetry.addData("value", gamepad1.left_stick_y);

            telemetry.update();

            //flip test
        }
    }
}