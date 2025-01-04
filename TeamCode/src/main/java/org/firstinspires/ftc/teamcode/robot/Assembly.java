package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.PIDController;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

//@TeleOp(name = "LM2 One Driver", group = "Test")
public class Assembly implements Subsystem {

    Servo flipLeft;
    Servo flipRight;

    Servo claw;
    final double clawOpen = 0.05;
    final double clawClose = 0.37;

    Servo rotateClaw;

    public double flipPos = 0;
    private Timer servoTimer;

    private int pathState;

    public static enum  STATE_VALUE {
        UNINITIALIZED,
        INITIALIZED,
        SLIDE_EXTENDING,
        SLIDE_EXTENDED,
        PITCH_EXTENDING,
        PITCH_EXTENDED,
        FLIP_EXTENDING,
        FLIP_EXTENDED
    }

    public STATE_VALUE curren_state = STATE_VALUE.UNINITIALIZED;

    // Pitch Stuff
    public int pitchTarget = 0;
    public  double kG = 0.027;
    public static double kG2 = 0.001;
    public static double kP = 0.005;
    public static  double kI = 0;
    public static  double kD = 0.00045;//pitch constant

    public DcMotorImplEx pitchMotor;
    PIDController pitchControl;

    public final int PITCH_LOW_POSITION = 300;
    public final int PITCH_MID_POSITION = 600;
    public final int PITCH_HIGH_POSITION = 1100;
    public final int PITCH_POSITION_TOLERANCE = 20;

    public enum PitchPosition {
        DOWN,
        MID,
        HIGH
    }


    // Slides Stuff
    public DcMotorImplEx slidesMotor;
    public double slidesKP = 0.005;
    public double slidesKI = 0; // slides constant
    public double slidesKD = 0.0008;
    public  double offset = 40;
    PIDController slidesControl;
    public  double slidesTarget = 0; //slides

    public  double ticksPerMaxExtend = 1936;
    public  double ticksPerRightAngle = 930;
    final double ticksToInches = (double) 26 /ticksPerMaxExtend;
    final double ticksToDegrees = (double) 90 /ticksPerRightAngle;

    public final int SLIDES_LOW_POSITION = 400;
    public final int SLIDES_MID_POSITION = 950;
    public final int SLIDES_HIGH_POSITION = 1750;
    public final int SLIDES_POSITION_TOLERANCE = 20;

    public final double FLIP_DOWN_POSITION = 0.130;
    public final double FLIP_UP_POSITION = 0.75;
    public final double FLIP_MID_POSITION = 0.575;
    public final double FLIP_LOW_MID_POSITION = 0.330;


    public enum SlidesPosition {
        DOWN,
        MID,
        HIGH,
        MANUALUP,
        MANUALDOWN,
        STAY
    }

    public Assembly(HardwareMap hardwareMap) {
        slidesMotor = hardwareMap.get(DcMotorImplEx.class, "slidesMotor");
        pitchMotor = hardwareMap.get(DcMotorImplEx.class, "pitchMotor");
        flipLeft = hardwareMap.get(Servo.class, "flipLeft");
        flipRight = hardwareMap.get(Servo.class, "flipRight");
        pitchMotor.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);
        pitchMotor.setMode(DcMotorImplEx.RunMode.RUN_WITHOUT_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flipLeft = hardwareMap.get(Servo.class, "flipLeft");
        flipRight = hardwareMap.get(Servo.class, "flipRight");
        rotateClaw = hardwareMap.get(Servo.class, "rotateClaw");
        claw = hardwareMap.get(Servo.class, "claw");

        slidesControl = new PIDController(slidesKP, slidesKI, slidesKD);
        pitchControl = new PIDController(kP, kI, kD);

        servoTimer = new Timer();
        curren_state = STATE_VALUE.INITIALIZED;
    }


    public Runnable flipClaw(double position) {
        flipLeft.setPosition(position);
        flipRight.setPosition(position * 0.94);
        curren_state = STATE_VALUE.FLIP_EXTENDED;
        servoTimer.resetTimer();
        return null;
    }

    public Runnable clawOpen() {
            claw.setPosition(clawOpen);
            return null;
    }

    public Runnable clawClose() {
            claw.setPosition(clawClose);
            return null;
    }

    public Runnable rotateClaw(double pos) {
            rotateClaw.setPosition(pos);
            return null;
    }

    public Runnable extendSlide(double position) {
        //ToDo need to repeat this because slides might not reach position in time,
        // want to repeat the code until it reaches the postion
        slidesTarget = position;
        slidesControl.setSetpoint(slidesTarget);
        double feedforward = kG * Math.sin(Math.toRadians((pitchMotor.getCurrentPosition() - offset) * ticksToDegrees)) + 0;
        double pid2 = slidesControl.calculate(slidesMotor.getCurrentPosition());
        slidesMotor.setPower(-(pid2 + feedforward));

        curren_state = STATE_VALUE.SLIDE_EXTENDING;
        return null;
    }

    public Runnable extendSlide(SlidesPosition position) {
        //ToDo need to repeat this because slides might not reach position in time,
        // want to repeat the code until it reaches the postion
        switch (position) {
            case DOWN:
                slidesTarget = SLIDES_LOW_POSITION;
                break;
            case MID:
                slidesTarget = SLIDES_MID_POSITION;
                break;
            case HIGH:
                slidesTarget = SLIDES_HIGH_POSITION;
                break;
            case MANUALUP:
                slidesTarget += 75;
                break;
            case MANUALDOWN:
                slidesTarget -= 75;
                break;
            case STAY:
                slidesTarget = slidesTarget;
                break;
        }
        extendSlide(slidesTarget);
        return null;
    }

    public Runnable anglePitch(int position) {
        //ToDo need to repeat this because slides might not reach position in time,
        // want to repeat the code until it reaches the postion
        pitchTarget = position;
        pitchControl.setSetpoint(pitchTarget);
        double feedforward2 = kG2 * (slidesMotor.getCurrentPosition() * ticksToInches) + 0;
        double feedforward3 = kG * Math.cos(Math.toRadians((pitchMotor.getCurrentPosition() - offset) * ticksToDegrees)) + 0;
        double pid = pitchControl.calculate(pitchMotor.getCurrentPosition());
        pitchMotor.setPower(feedforward3 + pid + feedforward2);

        double pos = pitchMotor.getCurrentPosition();
        curren_state= STATE_VALUE.PITCH_EXTENDING;
            //return pos>(pitchTarget - pitchPositionTolerance) && pos<(pitchTarget + pitchPositionTolerance);
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
        switch (curren_state) {
            case INITIALIZED:
                break;
            case UNINITIALIZED:
                break;
            case SLIDE_EXTENDING:
                double pos = slidesMotor.getCurrentPosition();

                if( pos>(slidesControl.getSetpoint() - SLIDES_POSITION_TOLERANCE) &&
                    pos<(slidesControl.getSetpoint() + SLIDES_POSITION_TOLERANCE)) {
                    curren_state = STATE_VALUE.SLIDE_EXTENDED;
                }
                else {
                    extendSlide(slidesControl.getSetpoint());
                }
                break;
            case SLIDE_EXTENDED:
                slidesControl.setSetpoint(slidesTarget);
                double feedforward = kG * Math.sin(Math.toRadians((pitchMotor.getCurrentPosition() - offset) * ticksToDegrees)) + 0;
                double pidSlides = slidesControl.calculate(slidesMotor.getCurrentPosition());
                slidesMotor.setPower(-(pidSlides + feedforward));
                break;
            case PITCH_EXTENDING:
                double pitch_pos = pitchMotor.getCurrentPosition();
                if( pitch_pos>(pitchControl.getSetpoint() - SLIDES_POSITION_TOLERANCE) &&
                        pitch_pos<(pitchControl.getSetpoint() + SLIDES_POSITION_TOLERANCE)) {
                    curren_state = STATE_VALUE.PITCH_EXTENDED;
                }
                else {
                    anglePitch((int)pitchControl.getSetpoint());
                }
                break;
            case PITCH_EXTENDED:
                pitchControl.setSetpoint(pitchTarget);
                double feedforward2 = kG2 * (slidesMotor.getCurrentPosition() * ticksToInches) + 0;
                double feedforward3 = kG * Math.cos(Math.toRadians((pitchMotor.getCurrentPosition() - offset) * ticksToDegrees)) + 0;
                double pidPitch = pitchControl.calculate(pitchMotor.getCurrentPosition());
                pitchMotor.setPower(feedforward3 + pidPitch + feedforward2);
                break;
            case FLIP_EXTENDING:
                if(servoTimer.getElapsedTimeSeconds() > 1.0) {
                    curren_state = STATE_VALUE.FLIP_EXTENDED;
                }
                break;
            case FLIP_EXTENDED:
                break;
        }
    }
}