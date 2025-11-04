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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous
@Disabled
public class ExampleAuto_ILERI extends LinearOpMode {
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState = 0;
    private Follower follower;

    public static Pose startingPose_REDGOAL = new Pose(121.64928909952606, 126.93838862559241, Math.toRadians(37)); //See ExampleAuto to understand how to use this
    public static Pose startingPose_CENTER = new Pose(72.08561236623068, 11.814506539833536, Math.toRadians(90)); //See ExampleAuto to understand how to use this

    public static Pose startingPose = startingPose_REDGOAL;

    private TelemetryManager telemetryM;

    public static PathChain line1, line2, line3;

    private ShooterSubsystem shooter;

    private DcMotor intakeMotor;
    private DcMotorEx shooterMotor;
    private int ticksPerRev = 28 * 1/2; // 28 / gear_ratio


    @Override
    public void runOpMode() throws InterruptedException {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        intakeMotor = hardwareMap.dcMotor.get("intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor = hardwareMap.get(DcMotorEx .class,"shooter");
        shooter = new ShooterSubsystem(shooterMotor,ticksPerRev);

        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        line1 = follower.pathBuilder().addPath(
                        // Path 1
                        new BezierLine(startingPose, new Pose(72.299, 72.150))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45)).build();

        line2 = follower.pathBuilder().addPath(
                        // Path 2
                        new BezierLine(new Pose(72.299, 72.150), new Pose(79.784, 80.083))
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45)).build();
        line3 = follower.pathBuilder()
                .addPath(
                        // Path 3
                        new BezierLine(new Pose(79.784, 80.083), new Pose(95.501, 95.501))
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
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
                shooter.setShooterTargetRPM(5000);
                follower.followPath(line1, 0.7, true);
                setPathState(1);
                break;

            case 1:
                if(!follower.isBusy()) {
                    sleep(3000);
                    follower.followPath(line2, true);
                    setPathState(2);
                }
            case 2:
                if(!follower.isBusy()) {
                    sleep(3000);
                    follower.followPath(line3, true);
                    setPathState(-1);
                }

            case -1:
                if (!follower.isBusy()) {
                    shooter.setShooterTargetRPM(0);
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