package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.modules.PIDController;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

public class Assembly implements Subsystem {

    Servo flipLeft;
    Servo flipRight;

    private Rev2mDistanceSensor distanceSensor;
    private AnalogInput pot;

    Servo claw;
    final double CLAW_OPEN = 0.43;
    final double CLAW_CLOSE = 0.72;

    Servo rotateClaw;

    public double flipPos = 0;
    private Timer servoTimer;

    private int pathState;

    public static enum  STATE_VALUE {
        UNINITIALIZED,
        INITIALIZED,
        SLIDE_EXTENDING,
        SLIDE_EXTENDED,
        PITCH_ROLLING,
        PITCH_ROLLED,
        FLIP_EXTENDING,
        FLIP_EXTENDED,
        FLIP_RETRACTING,
        FLIP_RETRACTED
    }

    public STATE_VALUE current_state = STATE_VALUE.UNINITIALIZED;

    // Pitch Stuff
    public double pitchTarget = 0;
    public static double kFPitch = 0.22;
    public static double kPPitch = 0.026;
    public static double kIPitch = 0;
    public static double kDPitch = 0.003;//pitch constant
    public static double lastTargetPitch = 0.0;
    public static double lastErrorPitch = 0.0;


    public DcMotorImplEx pitchMotor;
    PIDController pitchControl;

    public final int PITCH_LOW_POSITION = 10;
    public final int PITCH_MID_POSITION = 30;
    public final int PITCH_HIGH_POSITION = 110;
    public final int PITCH_POSITION_TOLERANCE = 1;

    public enum PitchPosition {
        DOWN,
        MID,
        HIGH
    }


    // Slides Stuff
    public DcMotorImplEx slidesMotor;
    ElapsedTime timerSlides = new ElapsedTime();
    public double slidesKP = 0.042;
    public double slidesKI = 0.001; // slides constant
    public double slidesKD = 0.002;
    public double slidesKF = 0.39;
    PIDController slidesControl;
    public double lastErrorSlides = 0.0;

    public  double ticksPerMaxExtend = 1936;
    public  double ticksPerRightAngle = 930;

    public final int SLIDES_LOW_POSITION = 5;
    public final int SLIDES_MID_POSITION = 30;
    public final int SLIDES_HIGH_POSITION = 55;
    public final int SLIDES_POSITION_TOLERANCE = 2;

    public final double FLIP_DOWN_POSITION = 0.136;
    public final double FLIP_UP_POSITION = 0.568;
    public final double FLIP_MID_POSITION = 0.375;

    protected ElapsedTime timerPitch = new ElapsedTime();


    public enum SlidesPosition {
        DOWN,
        MID,
        HIGH,
        MANUALUP,
        MANUALDOWN,
        STAY
    }

    private double rawToGoodWithFilterSlides(double pos) {
        double actualPos;
        if(pos>6) {actualPos = 0.0000330052 * (Math.pow(pos, 4)) -0.00356719 * (Math.pow(pos, 3)) + 0.137523 * (Math.pow(pos, 2))  -1.15156*pos + 9.04499;}
        else{actualPos = pos;}
        return Math.round(actualPos);
    }

    private double mapPotentiometerToAngle(double potentiometerValue) {
        return ((potentiometerValue - 0.47800000000000004)/ (1.1360000000000001-0.47800000000000004)) * (90 - 0) -0;
    }

    public Assembly(HardwareMap hardwareMap) {
        slidesMotor = hardwareMap.get(DcMotorImplEx.class, "slidesMotor");
        pitchMotor = hardwareMap.get(DcMotorImplEx.class, "pitchMotor");
        flipLeft = hardwareMap.get(Servo.class, "flipLeft");
        flipRight = hardwareMap.get(Servo.class, "flipRight");
        pitchMotor.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);
        pitchMotor.setMode(DcMotorImplEx.RunMode.RUN_WITHOUT_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //slidesMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        pitchMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        flipLeft = hardwareMap.get(Servo.class, "flipLeft");
        flipRight = hardwareMap.get(Servo.class, "flipRight");
        rotateClaw = hardwareMap.get(Servo.class, "rotateClaw");
        claw = hardwareMap.get(Servo.class, "claw");

        slidesControl = new PIDController(slidesKP, slidesKI, slidesKD);
        pitchControl = new PIDController(kPPitch, kIPitch, kDPitch);

        servoTimer = new Timer();
        current_state = STATE_VALUE.INITIALIZED;

        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distance");
        pot = hardwareMap.get(AnalogInput.class,"pot");
    }


    public Runnable flipClaw(double position) {
        flipLeft.setPosition(position);
        flipRight.setPosition(position * 0.94);
        if (position == FLIP_DOWN_POSITION) current_state = STATE_VALUE.FLIP_EXTENDING;
        if (position == FLIP_UP_POSITION) current_state = STATE_VALUE.FLIP_RETRACTING;
        servoTimer.resetTimer();
        return null;
    }

    public Runnable clawOpen() {
            claw.setPosition(CLAW_OPEN);
            return null;
    }

    public Runnable clawClose() {
            claw.setPosition(CLAW_CLOSE);
            return null;
    }

    public Runnable rotateClaw(double pos) {
            rotateClaw.setPosition(pos);
            return null;
    }

    public Runnable extendSlide(double position) {
        //ToDo need to repeat this because slides might not reach position in time,
        // want to repeat the code until it reaches the position
        double curPosRaw = distanceSensor.getDistance(DistanceUnit.CM);
        double actualCurPos = rawToGoodWithFilterSlides(curPosRaw);

        // Calculate error (using angles)
        double error = position - actualCurPos;
        double integralSum = 0;
        //ElapsedTime timerSlides = new ElapsedTime();

        integralSum += error * timerSlides.seconds();

        lastErrorSlides = error;

        double derivative = (error - lastErrorSlides) / timerSlides.seconds();

        double feedForward = slidesKF * Math.sin(Math.toRadians(mapPotentiometerToAngle(pot.getVoltage())));

        double motorPower = (slidesKP * error) + (slidesKI * integralSum) + (slidesKD * derivative) + feedForward;

        motorPower = Math.max(-1.0, Math.min(1.0, motorPower));

        slidesMotor.setPower(motorPower);

        timerSlides.reset();


        current_state = STATE_VALUE.SLIDE_EXTENDING;
        return null;
    }

    public Runnable anglePitch(double position) {
        //ToDo need to repeat this because slides might not reach position in time,
        // want to repeat the code until it reaches the postion
        //ElapsedTime timer = new ElapsedTime();

        double integralSum = 0.0;

        double currentVoltage = pot.getVoltage();
        double currentAngle = mapPotentiometerToAngle(currentVoltage);

        double targetAngle = position;
        // Calculate error (using angles)
        double error = targetAngle - currentAngle;

        if (targetAngle>lastTargetPitch){
            kPPitch = 0.026;
            kDPitch = 0.003;
            kIPitch = 0.000;
            kFPitch = 0.22;
        }else if (targetAngle<lastTargetPitch){
            kPPitch = 0.009;
            kDPitch = 0.0;
            kIPitch = 0.0;
            kFPitch= 0.1;
        }

        if(Math.abs(error) < 10 && Math.abs(error)>1){
//                kP = 0.054;
//                kD = 0.0015;
            kIPitch = 0.0018;
        }

        integralSum += error * timerPitch.seconds();

        double derivative = (error - lastErrorPitch) / timerPitch.seconds();

        double feedForward = kFPitch * Math.cos(Math.toRadians(currentAngle));

        double motorPower = (kPPitch * error) + (kIPitch * integralSum) + (kDPitch * derivative) + feedForward;

        motorPower = Math.max(-1.0, Math.min(1.0, motorPower));

        pitchMotor.setPower(motorPower);

        lastErrorPitch = error;
        lastTargetPitch = targetAngle;
        timerPitch.reset();

        current_state = STATE_VALUE.PITCH_ROLLING;
        return null;
    }

    public Runnable anglePitch(PitchPosition position) {
        //ToDo need to repeat this because slides might not reach position in time,
        // want to repeat the code until it reaches the postion
        switch (position) {
            case HIGH:
                pitchTarget = PITCH_HIGH_POSITION;
                break;
            case MID:
                pitchTarget = PITCH_MID_POSITION;
                break;
            case DOWN:
                pitchTarget = PITCH_LOW_POSITION;
                break;
        }
        anglePitch(pitchTarget);
        return null;
    }

    public void slidesManual(double value) {
        slidesMotor.setPower(value);
    }

    @Override
    public void update() {
        switch (current_state) {
            case INITIALIZED:
                break;
            case UNINITIALIZED:
                break;
            case SLIDE_EXTENDING:
                double pos = slidesMotor.getCurrentPosition();
/*
                if( pos>(slidesControl.getSetpoint() - SLIDES_POSITION_TOLERANCE) &&
                    pos<(slidesControl.getSetpoint() + SLIDES_POSITION_TOLERANCE)) {
                    current_state = STATE_VALUE.SLIDE_EXTENDED;
                }
                else {
                    extendSlide(slidesControl.getSetpoint());
                }

 */
                break;
            case SLIDE_EXTENDED:
                break;
            case PITCH_ROLLING:
                double pitch_pos = pitchMotor.getCurrentPosition();
                if( pitch_pos>(pitchControl.getSetpoint() - SLIDES_POSITION_TOLERANCE) &&
                        pitch_pos<(pitchControl.getSetpoint() + SLIDES_POSITION_TOLERANCE)) {
                    current_state = STATE_VALUE.PITCH_ROLLED;
                }
                else {
                    anglePitch((int)pitchControl.getSetpoint());
                }
                break;
            case PITCH_ROLLED:
                break;
            case FLIP_EXTENDING:
                if(servoTimer.getElapsedTimeSeconds() > 1.0) {
                    current_state = STATE_VALUE.FLIP_EXTENDED;
                }
                break;
            case FLIP_EXTENDED:
                break;
            case FLIP_RETRACTING:
                if(servoTimer.getElapsedTimeSeconds() > 1.0) {
                    current_state = STATE_VALUE.FLIP_RETRACTED;
                }
                break;
            case FLIP_RETRACTED:
                break;
        }
    }
}