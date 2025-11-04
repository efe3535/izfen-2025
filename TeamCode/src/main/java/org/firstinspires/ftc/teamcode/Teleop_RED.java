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
public class Teleop_RED extends OpMode {
    private Follower follower;
    public static Pose startingPose_BLUEGOAL = new Pose(121.64928909952606, 126.93838862559241, Math.toRadians(37)); //See ExampleAuto to understand how to use this
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

    private int id = 0;

    // Button state tracking for press detection
    private boolean lastAState = false;
    private boolean alignToAprilTag = false;

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

        leftLight = hardwareMap.get(Servo.class,"leftLight");
        rightLight = hardwareMap.get(Servo.class,"rightLight");


        /*shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setDirection(DcMotorEx.Direction.REVERSE);
        shooterMotor.setVelocityPIDFCoefficients(0.4, 0, 0, 12);*/
        shooter = new ShooterSubsystem(shooterMotor,ticksPerRev);
        linkage = new StackerSubsystem(stackerLeftMotor,stackerRightMotor,holderServo);


    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        linkage.hold();
        follower.startTeleopDrive(true);
    }

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
                            new Pose(96,96,Math.toRadians(45))
                    )
            ).setLinearHeadingInterpolation(
                    follower.getHeading(),
                    Math.toRadians(45)
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
                            new Pose(72,72,Math.toRadians(45))
                    )
            ).setLinearHeadingInterpolation(
                    follower.getHeading(),
                    Math.toRadians(45)
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
                            new Pose(72,24,Math.toRadians(59))
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
                            new Pose(72,108,Math.toRadians(26.5))
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

        if(gamepad1.left_stick_button) {
            linkage.feed();
        } else {
            linkage.hold();
        }


        // AprilTag alignment logic
        performAprilTagAlignment(result);

        if(gamepad1.x) {
            shooter.start();
            linkage.feed();
        } else {
            shooter.stop();
            linkage.hold();
        }

       /* if(gamepad1.dpad_left) {
            linkage.pull();
        } else {
            linkage.stop();
        }*/

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

        leftLight.setPosition(gamepad1.left_trigger);
        rightLight.setPosition(gamepad1.right_trigger);

        limelight.updateRobotOrientation(robotYaw);
        telemetry.addData("enkoder",shooterMotor.getCurrentPosition());
        telemetry.addData("motor RPM",(shooterMotor.getVelocity()/ticksPerRev)*60);
        telemetry.update();
    }

}