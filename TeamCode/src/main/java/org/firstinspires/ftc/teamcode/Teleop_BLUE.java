package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Configurable
@TeleOp
public class Teleop_BLUE extends OpMode {
    private Follower follower;
    public static Pose startingPose_BLUEGOAL = new Pose(144-121.64928909952606, 126.93838862559241, Math.toRadians(180-37)); //See ExampleAuto to understand how to use this
    public static Pose startingPose_CENTER = new Pose(72.08561236623068, 11.814506539833536, Math.toRadians(90)); //See ExampleAuto to understand how to use this

    public static PathChain line1;
    //public static Pose startingPose = startingPose_CENTER;
    public static Pose startingPose;
    private Limelight3A limelight = null;
    private DcMotor intakeMotor = null;
    private DcMotorEx shooterMotor = null;
    private DcMotor stackerLeftMotor = null;
    private DcMotor stackerRightMotor = null;
    private Servo holderServo = null;
    private Servo feederLeft = null;
    private Servo feederRight = null;

    // ------------------------- TUNING for servo test (ms) -------------------------
    // Change these values to tune the test durations
    public static final int TEST_FEED_MS = 200;   // how long feeders run during test (ms)
    public static final int TEST_HOLDER_MS = 200; // how long holder stays open during test (ms)
    // --------------------------------------------------------------------------------

    private Teleop_Constants teleopConstants;

    private Servo leftLight = null;
    private Servo rightLight = null;

    private ShooterSubsystem shooter;
    private StackerSubsystem linkage;
    //public static Pose startingPose = new Pose(72.08561236623068,11.814506539833536,Math.toRadians(37)); //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    public static double kP = 0.02;
    public static double shooterRPM = 1000;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private boolean startTurning = false;
    private double slowModeMultiplier = 0.5;
    private double APRILTAG_HEIGHT = 0.75;
    private double LIMELIGHT_HEIGHT = 0.292;
    private double LIMELIGHT_DISTANCE_FROM_END = 0.139;
    private int ticksPerRev = 28; // 28 / gear_ratio
    private double aprilTagX = 0, aprilTagY = 0;

    private boolean isThereArtifact = false;

    private int id = 0;

    // Button state tracking for press detection
    private boolean lastAState = false;
    private boolean alignToAprilTag = false;

    // Artifact detection debouncing
    private boolean artifactDetectedRaw = false;
    private long artifactFirstDetectedTime = 0;
    private final long ARTIFACT_DEBOUNCE_TIME_MS = 100;

    /**
     * Start AprilTag alignment correction for tag ID 20
     * This method can be called programmatically from code
     */
    public void correctAprilTag() {
        if (id == 20) {
            alignToAprilTag = true;
        }
    }

    /**
     * Stop AprilTag alignment correction
     */
    public void stopAprilTagCorrection() {
        alignToAprilTag = false;
        startTurning = false;
        follower.setTeleOpDrive(0, 0, 0, true);
    }

    /**
     * Check if AprilTag alignment is currently active
     */
    public boolean isAprilTagAligning() {
        return alignToAprilTag;
    }

    /**
     * Internal method that performs the actual AprilTag alignment logic
     */
    private void performAprilTagAlignment(LLResult result) {
        if (alignToAprilTag && id == 20) {
            if (result != null && result.isValid()) {
                // P controller parameters
                double kP = 0.015;               // proportional gain
                double minPower = 0.1;          // minimum turning power
                double maxPower = 0.45;         // maximum turning power
                double deadband = 2.0;          // deadband in degrees

                double error = aprilTagX;
                if (Math.abs(error) > deadband) {
                    double power = kP * error;
                    // Limit power and apply minimum
                    double absP = Math.min(Math.abs(power), maxPower);
                    absP = Math.max(absP, minPower);
                    power = Math.signum(power) * absP;

                    // Negative power for correct turning direction
                    follower.setTeleOpDrive(0, 0, -power, true);
                    startTurning = true;
                } else {
                    // Stop and disable alignment when aligned
                    startTurning = false;
                    alignToAprilTag = false;
                    follower.setTeleOpDrive(0, 0, 0, true);
                }
            } else {
                // Stop alignment when target is lost
                startTurning = false;
                alignToAprilTag = false;
            }
        }
    }

    /*
    public void setShooterTargetRPM(double targetRPM) {
        double ticksPerSecond = (targetRPM / 60.0) * ticksPerRev;
        shooterMotor.setVelocity(ticksPerSecond);
    }
    */



    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);

        follower = Constants.createFollower(hardwareMap);

        startingPose = PoseStorage.currentPose!=null ? PoseStorage.currentPose : startingPose_BLUEGOAL;

        follower.setStartingPose(startingPose);
        //follower.setPose(startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        intakeMotor = hardwareMap.dcMotor.get("intake");
        shooterMotor = hardwareMap.get(DcMotorEx.class,"shooter");

        stackerLeftMotor = hardwareMap.get(DcMotor.class,"floorLeft");
        stackerRightMotor = hardwareMap.get(DcMotor.class,"floorRight");
        holderServo = hardwareMap.get(Servo.class,"holderServo");
        // try to read feeder servos (optional)
        feederLeft = hardwareMap.get(Servo.class, "feederLeft");
        feederRight = hardwareMap.get(Servo.class, "feederRight");



        leftLight = hardwareMap.get(Servo.class,"leftLight");
        rightLight = hardwareMap.get(Servo.class,"rightLight");


        /*shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setDirection(DcMotorEx.Direction.REVERSE);
        shooterMotor.setVelocityPIDFCoefficients(0.4, 0, 0, 12);*/
        shooter = new ShooterSubsystem(shooterMotor,ticksPerRev);
        // construct linkage with feeders if available
        linkage = new StackerSubsystem(stackerLeftMotor, stackerRightMotor, feederLeft, feederRight, holderServo);


        // Test state machine fields
        // 0 = idle, 1 = feeders running, 2 = feeders stopped / open holder next, 3 = holder open
        // We'll use these fields (declared here) to avoid blocking the main loop
        // (fields declared after init to avoid changing file top layout heavily)
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        linkage.hold();
        follower.startTeleopDrive(true);
    }

    // test state machine runtime fields
    private int servoTestState = 0;
    private long servoTestStartMs = 0;
    private boolean lastLeftStickButton = false;

    @Override
    public void loop() {
        //Call this once per loop
        telemetryM.update();
        follower.update();
        //shooter.setShooterTargetRPM(shooterRPM);
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResultList = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducialResultList) {
                id = fiducial.getFiducialId();
                aprilTagX = fiducial.getTargetXDegrees();
                aprilTagY = fiducial.getTargetYDegrees();
            }
        }

        // Detect A button press (not hold)
        boolean currentAState = gamepad1.a;
        boolean aPressed = currentAState && !lastAState;
        lastAState = currentAState;

        // Left stick button rising edge triggers the servo test sequence
        boolean currentLeftStick = gamepad1.left_stick_button;
        boolean leftStickPressed = currentLeftStick && !lastLeftStickButton;
        lastLeftStickButton = currentLeftStick;

        if (leftStickPressed) {
            // start non-blocking servo test
            servoTestState = 1;
            servoTestStartMs = System.currentTimeMillis();
            telemetryM.debug("ServoTest", "started");
            // ensure starting positions: holder closed, feeders stopped
            if (linkage != null) {
                linkage.stopFeed();
                linkage.hold();
            }
        }

        // Start alignment mode when A is pressed (only start, don't toggle)
        if (aPressed && id == 20) {
            alignToAprilTag = true;
        }

        // Stop alignment when B is pressed
        if (gamepad1.b) {
            alignToAprilTag = false;
            startTurning = false;
            follower.setTeleOpDrive(0, 0, 0, true);
        }

        if(gamepad1.dpad_up) {
            shooter.setShooterTargetRPM(teleopConstants.DPAD_UP_TARGET);
            line1 = follower.pathBuilder().addPath(
                    new BezierLine(
                            follower.getPose(),
                            new Pose(48,96,Math.toRadians(135))
                    )
            ).setLinearHeadingInterpolation(
                    follower.getHeading(),
                    Math.toRadians(135)
            ).build(); //currentpose -> new Pose(72.299, 72.150)
            follower.followPath(line1);
            do {
                follower.update();
            }
            while(follower.isBusy());
            follower.startTeleopDrive(true);
        }

        if(gamepad1.dpad_right) {
            shooter.setShooterTargetRPM(teleopConstants.DPAD_RIGHT_TARGET);

            line1 = follower.pathBuilder().addPath(
                    new BezierLine(
                            follower.getPose(),
                            new Pose(72,72,Math.toRadians(135))
                    )
            ).setLinearHeadingInterpolation(
                    follower.getHeading(),
                    Math.toRadians(135)
            ).build(); //currentpose -> new Pose(72.299, 72.150)
            follower.followPath(line1);
            do {
                follower.update();
            }
            while(follower.isBusy());
            follower.startTeleopDrive(true);
            correctAprilTag();
        }

        if(gamepad1.dpad_down) {
            shooter.setShooterTargetRPM(teleopConstants.DPAD_DOWN_TARGET);
            line1 = follower.pathBuilder().addPath(
                    new BezierLine(
                            follower.getPose(),
                            new Pose(72,24,Math.toRadians(180 - 59))
                    )
            ).setLinearHeadingInterpolation(
                    follower.getHeading(),
                    Math.toRadians(180 - 59)
            ).build(); //currentpose -> new Pose(72.299, 72.150)
            follower.followPath(line1);
            do {
                follower.update();
            }
            while(follower.isBusy());
            follower.startTeleopDrive(true);
            correctAprilTag();
        }
        if(gamepad1.y) {
            shooter.setShooterTargetRPM(teleopConstants.Y_TARGET);
            line1 = follower.pathBuilder().addPath(
                    new BezierLine(
                            follower.getPose(),
                            new Pose(72,108,Math.toRadians(180 - 26.5))
                    )
            ).setLinearHeadingInterpolation(
                    follower.getHeading(),
                    Math.toRadians(180 - 26.5)
            ).build(); //currentpose -> new Pose(72.299, 72.150)
            follower.followPath(line1);
            do {
                follower.update();
            }
            while(follower.isBusy());
            follower.startTeleopDrive(true);
            correctAprilTag();
        }

        /*if(gamepad1.left_stick_button) {
            linkage.feed();
        } else {
            linkage.hold();
        }*/


        // AprilTag alignment logic
        performAprilTagAlignment(result);

        // Servo test state machine (non-blocking)
        if (servoTestState != 0) {
            long now = System.currentTimeMillis();
            if (servoTestState == 1) {
                // start feeders (if present)
                if (linkage != null) linkage.feed();
                servoTestStartMs = now;
                servoTestState = 2;
                telemetryM.debug("ServoTest", "feeding (state 1)");
            } else if (servoTestState == 2) {
                // wait TEST_FEED_MS then stop feeders and open holder
                if (now - servoTestStartMs >= TEST_FEED_MS) {
                    if (linkage != null) linkage.stopFeed();
                    // open holder briefly
                    if (linkage != null) linkage.openHolder();
                    servoTestStartMs = now;
                    servoTestState = 3;
                    telemetryM.debug("ServoTest", "holder opened (state 2)");
                }
            } else if (servoTestState == 3) {
                // wait TEST_HOLDER_MS then close holder and finish
                if (now - servoTestStartMs >= TEST_HOLDER_MS) {
                    if (linkage != null) linkage.hold();
                    servoTestState = 0;
                    telemetryM.debug("ServoTest", "completed");
                }
            }
        }

        if(gamepad1.x) {
            shooter.start();
            linkage.feed();
        } else {
            shooter.stop();
            linkage.hold();
        }

        if(gamepad1.left_bumper) {
            intakeMotor.setPower(1);
            linkage.pull();

        } else if(gamepad1.right_bumper)
        {
            intakeMotor.setPower(-1);
            linkage.repell();
        } else {
            intakeMotor.setPower(0);
            linkage.stop();
        }




        /*if(gamepad1.x) {
            floorLeft.setPower(1);
            floorRight.setPower(1);
            highLeft.setPower(1);
            highRight.setPower(1);
           // shooter.setShooterTargetRPM(5000);
        } else {
            floorLeft.setPower(0);
            floorRight.setPower(0);
            highLeft.setPower(0);
            highRight.setPower(0);
            shooter.setShooterTargetRPM(0);
        }*/

        /*if(gamepad1.y) {
            shooterMotor.setPower(0.5);
        } else {
            shooterMotor.setPower(0);
        }*/

        double robotYaw = follower.getHeading();
        if(!startTurning) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // Robot Centric
            );
        }


        //leftLight.setPosition(gamepad1.left_trigger);
        leftLight.setPosition(gamepad1.left_trigger);
        rightLight.setPosition(gamepad1.right_trigger);

        limelight.updateRobotOrientation(robotYaw);
        telemetry.addData("enkoder",shooterMotor.getCurrentPosition());
        telemetry.addData("motor RPM",(shooterMotor.getVelocity()/ticksPerRev)*60);
        telemetry.update();
    }

}