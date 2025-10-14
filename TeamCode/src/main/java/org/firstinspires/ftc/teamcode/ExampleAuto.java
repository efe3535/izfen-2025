package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.function.Supplier;

@Configurable
@Autonomous
public class ExampleAuto extends LinearOpMode {
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState = 0;
    private Follower follower;

    public static Pose startingPose_REDGOAL = new Pose(121.64928909952606, 126.93838862559241, Math.toRadians(37)); //See ExampleAuto to understand how to use this
    public static Pose startingPose_CENTER = new Pose(72.08561236623068, 11.814506539833536, Math.toRadians(90)); //See ExampleAuto to understand how to use this

    public static Pose startingPose = startingPose_REDGOAL;

    private TelemetryManager telemetryM;

    public static PathChain line1;
    public static PathChain line2;
    public static PathChain line3;
    public static PathChain line4;
    public static PathChain line5;
    public static PathChain line6;
    public static PathChain line7;
    public static PathChain line8;

    private DcMotor intakeMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        intakeMotor = hardwareMap.dcMotor.get("intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        line1 = follower.pathBuilder()
                .addPath(
                        // Path 1
                        new BezierLine(startingPose, new Pose(120.114, 119.431))
                )
                .setLinearHeadingInterpolation(startingPose.getHeading(), startingPose.getHeading()).build();
        line2 =
                follower.pathBuilder().addPath(
                                // Path 2
                                new BezierCurve(
                                        new Pose(120.114, 119.431),
                                        new Pose(62.445, 105.441),
                                        new Pose(85.479, 83.261)
                                )
                        )
                        .setLinearHeadingInterpolation(startingPose.getHeading(), Math.toRadians(0)).build();

        line3 = follower.pathBuilder().addPath(
                        // Path 3
                        new BezierLine(new Pose(85.479, 83.261), new Pose(122.844, 83.261))
                )
                .setTangentHeadingInterpolation().build();
        line4 = follower.pathBuilder().addPath(
                        // Path 4
                        new BezierCurve(
                                new Pose(122.844, 83.261),
                                new Pose(97.422, 75.754),
                                new Pose(85.308, 60.227)
                        ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();
        line5 = follower.pathBuilder().addPath(
                        // Path 5
                        new BezierLine(new Pose(85.308, 60.227), new Pose(123.185, 60.227))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();
        line6 = follower.pathBuilder().addPath(
                        // Path 6
                        new BezierCurve(
                                new Pose(123.185, 60.227),
                                new Pose(95.886, 52.550),
                                new Pose(91.109, 36.171)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();
        line7 = follower.pathBuilder().addPath(
                        // Path 7
                        new BezierLine(new Pose(91.109, 36.171), new Pose(130.009, 36.000))
                )
                .setTangentHeadingInterpolation().build();

        line8 = follower.pathBuilder().addPath(
                        // Path 8
                        new BezierCurve(
                                new Pose(130.009, 36.000),
                                new Pose(85.649, 11.261),
                                new Pose(84.455, 67.393),
                                new Pose(38.218, 33.441)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        /*
        line1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(startingPose, new Pose(114.549, 113.008))
                )
                .setLinearHeadingInterpolation(Math.toRadians(startingPose.getHeading()), Math.toRadians(35))
                .build();

        line2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(114.549, 113.008),
                                new Pose(49.484, 59.929),
                                new Pose(99.310, 59.586)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        line3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(99.310, 59.586), new Pose(121.570, 59.244))
                )
                .setTangentHeadingInterpolation()
                .build();
                */

        waitForStart();
        if (isStopRequested()) return;

        //autonomousPathUpdate();

        while (opModeIsActive()) {
            follower.update();
            autonomousPathUpdate();

            telemetryM.update();
            telemetryM.debug("heading", follower.getHeading());
            telemetryM.debug("position", follower.getPose());
            telemetryM.debug("velocity", follower.getVelocity());
            telemetryM.debug("Busy?", follower.isBusy());
            telemetryM.debug("PathState", pathState);
        }

    }

    public void autonomousPathUpdate() {
        // line 3 5 7
        switch (pathState) {
            case 0:
                follower.followPath(line1, true);
                setPathState(1);
                break;
            case 1:
            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(line2, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    intakeMotor.setPower(1);
                    follower.followPath(line3, 0.5, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    intakeMotor.setPower(0);
                    follower.followPath(line4, true);
                    setPathState(4);
                }
                break;
            case 4:
            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    intakeMotor.setPower(1);
                    follower.followPath(line5, 0.5, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    intakeMotor.setPower(0);
                    follower.followPath(line6, 0.3, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    intakeMotor.setPower(1);
                    follower.followPath(line7, 0.5, true);
                    setPathState(7);
                }
                break;
            case 7:
            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    intakeMotor.setPower(0);
                    follower.followPath(line8, true);
                    setPathState(-1);
                }
                break;

            case -1:
                if (!follower.isBusy()) {
                    intakeMotor.setPower(0);
                }

        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

}