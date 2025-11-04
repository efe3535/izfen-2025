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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous
public class Auto_BLUE extends LinearOpMode {
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
    private StackerSubsystem stacker;

    // ------------------------- TUNING (ms, ratios, counts) -------------------------
    // Change these values when testing to tune timing and thresholds
    public static final int TUNING_BALLS_TO_SHOOT = 3;                // how many balls per goAndShoot
    public static final int TUNING_LINKAGE_MS = 700;                  // ms to run linkage (stacker.pull())
    public static final int TUNING_FEED_PULSE_MS = 200;               // ms to run feeders or open holder
    public static final int TUNING_FEEDERS_TO_HOLDER_DELAY_MS = 150;  // ms between feeders pushing and opening holder (3rd ball)
    public static final int TUNING_HOLDER_OPEN_MS = 100;              // ms to keep holder open for 3rd ball
    public static final long TUNING_SENSOR_TIMEOUT_MS = 2000L;        // ms timeout waiting for sensor confirmation
    public static final double TUNING_SHOOTER_READY_RATIO = 0.85;     // ratio of target RPM to consider shooter ready
    // --------------------------------------------------------------------------------

    @Override
    public void runOpMode() throws InterruptedException {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        intakeMotor = hardwareMap.dcMotor.get("intake");
        shooterMotor = hardwareMap.get(DcMotorEx.class,"shooter");
        shooter = new ShooterSubsystem(shooterMotor, ticksPerRev);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DcMotor stackerLeft = hardwareMap.dcMotor.get("stackerLeft");
        DcMotor stackerRight = hardwareMap.dcMotor.get("stackerRight");
        // New hardware: two feeder servos and one holder servo
        Servo feederLeft = null;
        Servo feederRight = null;
        Servo holder = null;
        try {
            feederLeft = hardwareMap.servo.get("feederLeft");
        } catch (Exception ignored) { feederLeft = null; }
        try {
            feederRight = hardwareMap.servo.get("feederRight");
        } catch (Exception ignored) { feederRight = null; }
        try {
            holder = hardwareMap.servo.get("holderServo");
        } catch (Exception ignored) { holder = null; }

        stacker = new StackerSubsystem(stackerLeft, stackerRight, feederLeft, feederRight, holder);

        // report which stacker servos were found (helps debugging hardware config)
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetryM.debug("StackerHW", "feederLeft=" + (feederLeft != null) + ", feederRight=" + (feederRight != null) + ", holder=" + (holder != null));

        opmodeTimer.resetTimer();
        // initialize telemetry early so we can report hardware attach issues
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();

        // try to attach a Rev2m or other distance sensor to the stacker for ball detection
        try {
            // NOTE: change the name "rev2m" below to the actual sensor name in your configuration
            com.qualcomm.robotcore.hardware.DistanceSensor rev2m = hardwareMap.get(com.qualcomm.robotcore.hardware.DistanceSensor.class, "distance");
            stacker.setDistanceSensor(rev2m);
        } catch (Exception e) {
            telemetryM.debug("StackerSensor", "rev2m not found or failed to attach");
        }

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


        // initialize stacker (adjust hardware names to your config)


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

        // reset stacker sensor/debounce state so each goAndShoot call starts fresh
        if (stacker != null) {
            stacker.resetDebounceFromCurrent();
            stacker.hold();
        }

        shooter.setShooterTargetRPM(Teleop_Constants.DPAD_UP_TARGET);
        shooter.start();
        follower.followPath(shoot1, 0.7, true);
        while (follower.isBusy() && opModeIsActive()) {
            follower.update();
        }

        // sensor-driven feeding: increment counter on each detected fired ball (falling edge)
        final int BALLS_TO_SHOOT = TUNING_BALLS_TO_SHOOT;
        final int LINKAGE_MILLIS = TUNING_LINKAGE_MS; // short pulse to lift ball to sensor area
        final int FEED_PULSE_MILLIS = TUNING_FEED_PULSE_MS; // servo pulse time
        final long SENSOR_TIMEOUT_MS = TUNING_SENSOR_TIMEOUT_MS; // fallback timeout while waiting for sensor (ms)

        int ballsFired = 0;
        while (ballsFired < BALLS_TO_SHOOT && opModeIsActive()) {
            // lift the next ball into place
            if (stacker != null) {
                stacker.pull();
                sleep(LINKAGE_MILLIS);
                stacker.stop();
            } else {
                intakeMotor.setPower(1.0);
                sleep(LINKAGE_MILLIS);
                intakeMotor.setPower(0.0);
            }

            // wait for shooter RPM >= TUNING_SHOOTER_READY_RATIO * target before feeding
            double target = 0.0;
            try { target = shooter.getShooterTargetRPM(); } catch (Exception ignored) {}
            if (target > 0.0) {
                while (opModeIsActive()) {
                    double currentRPM = 0.0;
                    try { currentRPM = shooter.getRPM(); } catch (Exception ignored) { currentRPM = Double.MAX_VALUE; }
                    if (currentRPM >= target * TUNING_SHOOTER_READY_RATIO) break;
                    // keep updating sensor while waiting so we don't miss events
                    if (stacker != null && stacker.updateSensor()) {
                        // unlikely to register a shot before feeding, but handle defensively
                        ballsFired++;
                        telemetryM.debug("BallFired", "detected early during rpm wait. count=" + ballsFired);
                        if (ballsFired >= BALLS_TO_SHOOT) break;
                    }
                    idle();
                }
            } else {
                // if no target info, short idle while still updating sensor
                long t0 = System.currentTimeMillis();
                while (System.currentTimeMillis() - t0 < 50 && opModeIsActive()) {
                    if (stacker != null && stacker.updateSensor()) {
                        ballsFired++;
                        telemetryM.debug("BallFired", "detected early during short wait. count=" + ballsFired);
                        break;
                    }
                    idle();
                }
            }

            if (ballsFired >= BALLS_TO_SHOOT) break;

            // trigger feed action (servo). For every 3rd ball we open holder and run feeders, otherwise a small feeder pulse.
            // New behavior:
            // - For balls 1 and 2: DO NOT run feeders. Keep feeders neutral and just open holder briefly to let the ball into the shooter.
            // - For ball 3: run feeders first (push), then open holder so the pushed ball enters the shooter. Stop feeders and close holder afterwards.
            final int FEEDERS_TO_HOLDER_DELAY_MS = TUNING_FEEDERS_TO_HOLDER_DELAY_MS; // short delay to let feeders push before opening holder (tune as needed)
            if ((ballsFired + 1) % 3 == 0) {
                // 3rd ball: run feeders then open holder
                if (stacker != null) {
                    telemetryM.debug("Feeding", "3rd ball: running feeders then opening holder");
                    // ensure holder is closed while feeders push
                    stacker.hold();
                    stacker.feed();
                    sleep(FEED_PULSE_MILLIS);
                    // small wait for ball to move into position
                    sleep(FEEDERS_TO_HOLDER_DELAY_MS);
                    // open holder so ball can enter shooter
                    stacker.openHolder();
                    sleep(TUNING_HOLDER_OPEN_MS); // short open time for the ball to pass
                    // stop feeders and close holder
                    stacker.stopFeed();
                    stacker.hold();
                    telemetryM.debug("Feeding", "3rd ball feed complete");
                } else {
                    // fallback: pulse intake then short wait
                    telemetryM.debug("Feeding", "fallback: intake pulse for 3rd ball");
                    intakeMotor.setPower(1.0);
                    sleep(FEED_PULSE_MILLIS + FEEDERS_TO_HOLDER_DELAY_MS);
                    intakeMotor.setPower(0.0);
                }
            } else {
                // balls 1 and 2: do NOT run feeders. Open holder briefly to allow the ball into the shooter.
                if (stacker != null) {
                    telemetryM.debug("Feeding", "normal ball: open holder briefly (no feeders)");
                    // ensure feeders are neutral
                    stacker.stopFeed();
                    // open holder to let the ball in
                    stacker.openHolder();
                    sleep(FEED_PULSE_MILLIS);
                    // close holder again
                    stacker.hold();
                    telemetryM.debug("Feeding", "normal feed complete (holder only)");
                } else {
                    // fallback: pulse intake
                    telemetryM.debug("Feeding", "fallback: intake pulse for normal ball");
                    intakeMotor.setPower(1.0);
                    sleep(FEED_PULSE_MILLIS);
                    intakeMotor.setPower(0.0);
                }
            }

            // After feeding, wait for the sensor falling edge to confirm the ball left, increment counter.
            long startWait = System.currentTimeMillis();
            boolean counted = false;
            while (opModeIsActive() && !counted) {
                if (stacker != null && stacker.updateSensor()) {
                    ballsFired++;
                    counted = true;
                    telemetryM.debug("BallFired", "confirmed by sensor. count=" + ballsFired);
                    break;
                }
                // fallback timeout: if sensor doesn't detect within SENSOR_TIMEOUT_MS, increment anyway and warn
                if (System.currentTimeMillis() - startWait > SENSOR_TIMEOUT_MS) {
                    ballsFired++;
                    telemetryM.debug("BallFired", "sensor timeout, incrementing anyway. count=" + ballsFired);
                     break;
                 }
                 idle();
             }

            // small idle to separate balls
            sleep(120);
        }

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
                    // start intake+linkage to store balls internally (do NOT feed shooter)
                    startIntakeStorage();
                     follower.followPath(line3, 0.65, true);
                     setPathState(3);
                 }
                 break;
            case 3:
                if (!follower.isBusy()) {
                    // stop intake and linkage when arriving at shooting point
                    stopIntakeStorage();
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
                    // start intake+linkage to store balls internally (do NOT feed shooter)
                    startIntakeStorage();
                    follower.followPath(line5, 0.5, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    // stop intake and linkage before shooting
                    stopIntakeStorage();
                    goAndShoot(follower.getPose());
                     //follower.followPath(line6, 0.3, true);
                     setPathState(6);
                 }
                 break;
             case 6:
                 if (!follower.isBusy()) {
                    // start intake+linkage to store balls internally (do NOT feed shooter)
                    startIntakeStorage();
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
                    // stop intake and linkage before shooting
                    stopIntakeStorage();
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

    // Helper to start intake + linkage to store balls (autonomous intake behavior)
    private void startIntakeStorage() {
        if (intakeMotor != null) intakeMotor.setPower(1);
        if (stacker != null) {
            stacker.hold(); // ensure holder closed so ball doesn't enter shooter
            stacker.pull();
        }
    }

    // Helper to stop intake + linkage
    private void stopIntakeStorage() {
        if (intakeMotor != null) intakeMotor.setPower(0);
        if (stacker != null) {
            stacker.stop();
            stacker.hold();
        }
    }

}
