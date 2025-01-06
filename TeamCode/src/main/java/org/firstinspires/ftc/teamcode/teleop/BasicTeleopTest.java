package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftBackMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftFrontMotorDirection;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftRearMotorDirection;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightBackMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightFrontMotorDirection;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightRearMotorDirection;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.PoseUpdater;
import org.firstinspires.ftc.teamcode.pedroPathing.util.DashboardPoseTracker;
import org.firstinspires.ftc.teamcode.robot.Assembly;

import java.util.List;

@TeleOp(name = "Basic Teleop Test", group = "Test")
public class BasicTeleopTest extends LinearOpMode {

    private Assembly assembly;


    private DcMotorEx leftFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightFront;
    private DcMotorEx rightBack;
    private List<DcMotorEx> motors;

    @Override
    public void runOpMode() throws InterruptedException {
        while(opModeInInit()){
            assembly = new Assembly(hardwareMap);

            leftFront = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
            leftBack = hardwareMap.get(DcMotorEx.class, leftBackMotorName);
            rightBack = hardwareMap.get(DcMotorEx.class, rightBackMotorName);
            rightFront = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);
            leftFront.setDirection(leftFrontMotorDirection);
            leftBack.setDirection(leftRearMotorDirection);
            rightFront.setDirection(rightFrontMotorDirection);
            rightBack.setDirection(rightRearMotorDirection);
        }

        waitForStart();

        while(!isStopRequested()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x; // this is strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double leftFrontPower = (y + x + rx) / denominator;
            double leftRearPower = (y - x + rx) / denominator;
            double rightFrontPower = (y - x - rx) / denominator;
            double rightRearPower = (y + x - rx) / denominator;

            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftRearPower);
            rightFront.setPower(rightFrontPower);
            rightBack.setPower(rightRearPower);




            if (gamepad1.dpad_up){
                assembly.extendSlide(50);
            }else if(gamepad1.dpad_right){
                assembly.extendSlide(30);
            }else if(gamepad1.dpad_down){
                assembly.extendSlide(20);
            }



            if (gamepad1.cross){
                assembly.anglePitch(15);
            } else if (gamepad1.triangle) {
                assembly.anglePitch(87);
            }else if (gamepad1.circle){
                assembly.anglePitch(50);
            }




            if (gamepad1.right_trigger > 0.1){
                assembly.flipClaw(assembly.FLIP_UP_POSITION);
            }else if(gamepad1.left_trigger > 0.1){
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