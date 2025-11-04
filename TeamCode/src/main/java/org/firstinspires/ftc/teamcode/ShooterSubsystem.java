package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class ShooterSubsystem {
    DcMotorEx shooterMotor;
    private int ticksPerRev = 28; // 28 / gear_ratio
    private double shooterTarget = Teleop_Constants.TARGET_RPM_DEFAULT;

    public ShooterSubsystem(DcMotorEx shooterMotor, int ticksPerRev) {
        this.shooterMotor = shooterMotor;
        this.ticksPerRev = ticksPerRev;
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setDirection(DcMotorEx.Direction.REVERSE);
        //shooterMotor.setVelocityPIDFCoefficients(0.4, 0, 0, 12);
        shooterMotor.setVelocityPIDFCoefficients(0.8, 0, 0, 12);
    }
    public void setShooterTargetRPM(double targetRPM) {
        shooterTarget = targetRPM;
    }

    public void start() {
        double ticksPerSecond = (shooterTarget / 60.0) * ticksPerRev;
        shooterMotor.setVelocity(ticksPerSecond);
    }

    public void stop() {
        shooterMotor.setVelocity(0);
    }

    public double getRPM() {
        return (shooterMotor.getVelocity()/ticksPerRev)*60;
    }

    public double getShooterTargetRPM() {
        return shooterTarget;
    }
}
