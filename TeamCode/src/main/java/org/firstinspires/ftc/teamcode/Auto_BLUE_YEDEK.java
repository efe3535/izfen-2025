package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous
public class Auto_BLUE_YEDEK extends LinearOpMode {
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState = 0;
    private Follower follower;

    public static Pose startingPose_BLUEGOAL = new Pose(22.351, 126.93838862559241, Math.toRadians(180 - 37)); //See ExampleAuto to understand how to use this
    public static Pose startingPose_CENTER = new Pose(72.08561236623068, 11.814506539833536, Math.toRadians(90)); //See ExampleAuto to understand how to use this

    public static Pose startingPose = startingPose_BLUEGOAL;

    private TelemetryManager telemetryM;

    public static PathChain line1;
    public static PathChain line2;
    public static PathChain line3;
    public static PathChain line4;
    public static PathChain line5;
    public static PathChain line6;
    public static PathChain line7;
    public static PathChain line8;

    public static PathChain shoot1;
    public static PathChain shoot2;
    public static PathChain shoot3;

    public static int DELAY = 500;

    private int ticksPerRev = 28; // 28 / gear_ratio

    private DcMotor intakeMotor;
    private DcMotorEx shooterMotor;
    private ShooterSubsystem shooter;

    @Override
    public void runOpMode() throws InterruptedException {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        intakeMotor = hardwareMap.dcMotor.get("intake");
        shooterMotor = hardwareMap.get(DcMotorEx.class,"shooter");
        shooter = new ShooterSubsystem(shooterMotor, ticksPerRev);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        shoot1 = follower.pathBuilder().addPath(
                        // Path 1
                        new BezierLine(startingPose, new Pose(144-72.299, 72.150))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-90), Math.toRadians(180-45)).build();

        shoot2 = follower.pathBuilder().addPath(
                        // Path 2
                        new BezierLine(new Pose(144-72.299, 72.150), new Pose(144-79.784, 80.083))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-45), Math.toRadians(180-45)).build();
        shoot3 = follower.pathBuilder()
                .addPath(
                        // Path 3
                        new BezierLine(new Pose(144-79.784, 80.083), new Pose(144-95.501, 95.501))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-45), Math.toRadians(180-45))
                .build();

        line1 = follower.pathBuilder()
                .addPath(
                        // Path 1
                        new BezierLine(startingPose, new Pose(144-120.114, 119.431))
                )
                .setLinearHeadingInterpolation(startingPose.getHeading(), startingPose.getHeading()).build();
        line2 =
                follower.pathBuilder().addPath(
                                // Path 2
                                new BezierCurve(
                                        new Pose(144-120.114, 119.431),
                                        new Pose(144-62.445, 105.441),
                                        new Pose(144-85.479, 83.261)
                                )
                        )
                        .setLinearHeadingInterpolation(startingPose.getHeading(), Math.toRadians(180-0)).build();

        line3 = follower.pathBuilder().addPath(
                        // Path 3
                        new BezierLine(new Pose(144-85.479, 83.261), new Pose(144-122.844, 83.261))
                )
                .setTangentHeadingInterpolation().build();
        line4 = follower.pathBuilder().addPath(
                        // Path 4
                        new BezierCurve(
                                new Pose(144-122.844, 83.261),
                                new Pose(144-97.422, 75.754),
                                new Pose(144-85.308, 60.227)
                        ))
                .setLinearHeadingInterpolation(Math.toRadians(180-0), Math.toRadians(180-0)).build();
        line5 = follower.pathBuilder().addPath(
                        // Path 5
                        new BezierLine(new Pose(144-85.308, 60.227), new Pose(144-123.185, 60.227))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-0), Math.toRadians(180-0)).build();
        line6 = follower.pathBuilder().addPath(
                        // Path 6
                        new BezierCurve(
                                new Pose(144-123.185, 60.227),
                                new Pose(144-95.886, 52.550),
                                new Pose(144-91.109, 36.171)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-0), Math.toRadians(180-0)).build();
        line7 = follower.pathBuilder().addPath(
                        // Path 7
                        new BezierLine(new Pose(144-91.109, 36.171), new Pose(144-130.009, 36.000))
                )
                .setTangentHeadingInterpolation().build();

        line8 = follower.pathBuilder().addPath(
                        // Path 8
                        new BezierLine(
                                new Pose(144-130.009, 36.000),
                                new Pose(48, 72)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-0), Math.toRadians(180-180))
                .build();
        /*
        line1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(startingPose, new Pose(114.549, 113.008))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-startingPose.getHeading()), Math.toRadians(180-35))
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
            PoseStorage.currentPose = follower.getPose();

            telemetryM.update();
            telemetryM.debug("heading", follower.getHeading());
            telemetryM.debug("position", follower.getPose());
            telemetryM.debug("velocity", follower.getVelocity());
            telemetryM.debug("Busy?", follower.isBusy());
            telemetryM.debug("PathState", pathState);
        }

    }

    public void goAndShoot(Pose start) {
        shoot1 = follower.pathBuilder().addPath(
                        // Path 1
                        new BezierLine(start, new Pose(144-96, 96))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-start.getHeading()), Math.toRadians(180-45)).build();
        shooter.setShooterTargetRPM(Teleop_Constants.DPAD_UP_TARGET);
        shooter.start();
        follower.followPath(shoot1, 0.7, true);
        while (follower.isBusy() && opModeIsActive()) {
            follower.update();
        }
        sleep(3000);
        /*sleep(DELAY);
        follower.followPath(shoot2, true);
        while (follower.isBusy() && opModeIsActive()) {
            follower.update();
        }
        sleep(DELAY);
        follower.followPath(shoot3, true);
        while (follower.isBusy() && opModeIsActive())  {
            follower.update();
        }*/
        shooter.stop();
    }

    public void autonomousPathUpdate() {
        // line 3 5 7
        switch (pathState) {
            case 0:
                //follower.followPath(line1, true);
                goAndShoot(follower.getPose());
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
                    //follower.followPath(line2, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    intakeMotor.setPower(1);
                    follower.followPath(line3, 0.65, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    intakeMotor.setPower(0);
                    goAndShoot(follower.getPose());

//                    follower.followPath(line4, true);
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
                    goAndShoot(follower.getPose());
                    //follower.followPath(line6, 0.3, true);
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
                    goAndShoot(follower.getPose());
                    follower.followPath(line8, true);
                    PoseStorage.currentPose = follower.getPose();
                    setPathState(-1);
                }
                break;

            case -1:
                if (!follower.isBusy()) {
                    intakeMotor.setPower(0);
                }
                break;

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