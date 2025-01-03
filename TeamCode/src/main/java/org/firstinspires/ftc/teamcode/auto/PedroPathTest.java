package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Autonomous(name="Pedro Path Test", group="Auto Test")
public class PedroPathTest extends LinearOpMode {
    Follower follower;
    int pathState;

    private final Pose startPose = new Pose(7.3, 104.7, 0);

    private final Pose preloadPose = new Pose(15.8, 126.2, Math.toRadians(135));
    private final Pose controlPreload1 = new Pose(36.8, 104.7, Math.toRadians(135));
    private final Pose controlPreload2 = new Pose(25.3, 117.1, Math.toRadians(135));

    private PathChain scorePreload;

    public void buildPaths(){
        scorePreload = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(startPose), new Point(preloadPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), preloadPose.getHeading())
                .build();


    }

    public void autoPathUpdate(){
        switch (pathState){
            case 0:
                follower.followPath(scorePreload);
                break;

        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        while (opModeInInit()){
            follower = new Follower(hardwareMap);
            follower.setStartingPose(startPose);
            buildPaths();
        }

        waitForStart();


        while(!isStopRequested()){
            autoPathUpdate();
            follower.update();
        }
    }
}
