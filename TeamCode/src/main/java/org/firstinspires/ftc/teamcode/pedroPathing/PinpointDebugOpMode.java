package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class PinpointDebugOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        waitForStart();
        while (opModeIsActive()) {
            pinpoint.update();

            telemetry.addData("Encoder X", pinpoint.getEncoderX());
            telemetry.addData("Encoder Y", pinpoint.getEncoderY());
            telemetry.addData("Heading (deg)", pinpoint.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }
    }
}