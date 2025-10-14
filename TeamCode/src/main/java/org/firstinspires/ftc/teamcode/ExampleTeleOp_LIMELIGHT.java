package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;
import java.util.function.Supplier;

@Configurable
@TeleOp
public class ExampleTeleOp_LIMELIGHT extends OpMode {
    private Follower follower;
    public static Pose startingPose = new Pose(71.48815165876778, 10.236966824644544, Math.toRadians(90)); //See ExampleAuto to understand how to use this
    private Limelight3A limelight = null;
    //public static Pose startingPose = new Pose(72.08561236623068,11.814506539833536,Math.toRadians(37)); //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    private double APRILTAG_HEIGHT = 0.75;
    private double LIMELIGHT_HEIGHT = 0.292;
    private double LIMELIGHT_DISTANCE_FROM_END = 0.139;
    private double aprilTagX = 0, aprilTagY = 0;

    private double calculateDistance(double degree) {
        return (APRILTAG_HEIGHT - LIMELIGHT_HEIGHT) / Math.tan(Math.toRadians(degree)) - LIMELIGHT_DISTANCE_FROM_END;
    }

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierCurve(
                        new Pose(72.086, 11.815),
                        new Pose(72.086, 103.591),
                        new Pose(26.369, 116.090)
                )))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(140))
                .build();
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive(true);

    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true // Robot Centric
        );


        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx();
            double ty = result.getTy();
            double ta = result.getTa();
            List<LLResultTypes.FiducialResult> fiducialResultList = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducialResultList) {
                int id = fiducial.getFiducialId();
                aprilTagX = fiducial.getTargetXDegrees();
                aprilTagY = fiducial.getTargetYDegrees();

                telemetryM.debug("distance", calculateDistance(ty) + "m");
                telemetryM.debug("id", id);
            }
            telemetryM.debug("tx", tx);
            telemetryM.debug("ty", ty);
            telemetryM.debug("ta", ta);
        }

        double robotYaw = follower.getHeading();
        limelight.updateRobotOrientation(robotYaw);
        telemetry.update();
    }
}