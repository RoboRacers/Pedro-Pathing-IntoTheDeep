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

    @Override
    public void runOpMode() throws InterruptedException {
        while(opModeInInit()){
            assembly = new Assembly(hardwareMap);
        }

        waitForStart();

        while(!isStopRequested()) {
            if (gamepad1.dpad_up){
                assembly.extendSlide(assembly.SLIDES_HIGH_POSITION);
            }else if(gamepad1.dpad_right){
                assembly.extendSlide(assembly.SLIDES_MID_POSITION);
            }else if(gamepad1.dpad_left){
                assembly.extendSlide(assembly.SLIDES_LOW_POSITION);
            }

            if (gamepad1.triangle){
                assembly.flipClaw(assembly.FLIP_UP_POSITION);
            }else if(gamepad1.cross){
                assembly.flipClaw(assembly.FLIP_DOWN_POSITION);
            }

            if(gamepad1.right_bumper){
                assembly.clawOpen();
            } else if (gamepad1.left_bumper) {
                assembly.clawClose();
            }

            telemetry.addData("Slides", assembly.slidesTarget);


            telemetry.update();
        }
    }
}