/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class StackerSubsystem {
    DcMotor stackerLeftMotor;
    DcMotor stackerRightMotor;
    Servo holderServo;
    private double REAR_STACKER_SPEED = 0.8, FRONT_STACKER_SPEED = 0.8;

    public StackerSubsystem(DcMotor stackerLeftMotor,DcMotor stackerRightMotor,Servo holderServo) {
        this.stackerLeftMotor = stackerLeftMotor;
        this.stackerRightMotor = stackerRightMotor;
        this.holderServo = holderServo;
        stackerLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        stackerRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        stackerLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        stackerRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        holderServo.setDirection(Servo.Direction.FORWARD);
    }

    public void pull() {
        stackerLeftMotor.setPower(FRONT_STACKER_SPEED);
        stackerRightMotor.setPower(REAR_STACKER_SPEED);
    }
    public void repell() {
        stackerLeftMotor.setPower(-FRONT_STACKER_SPEED);
        stackerRightMotor.setPower(-REAR_STACKER_SPEED);
    }

    public void hold() {
        holderServo.setPosition(0);
    }

    public void feed() {
        holderServo.setPosition(0.05);
    }

    public void stop() {
        stackerLeftMotor.setPower(0);
        stackerRightMotor.setPower(0);
    }


}
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class StackerSubsystem {
    DcMotor stackerLeftMotor;
    DcMotor stackerRightMotor;
    // now: two feeder servos that push the ball (like pinball)
    private Servo feederLeftServo;
    private Servo feederRightServo;
    // single holder servo that blocks/lets balls into the shooter
    private Servo holderServo;

    // ------------------------- TUNING (change these when testing) -------------------------
    // Distance sensor threshold (mm) and debounce (ms)
    public static double TUNING_PRESENCE_THRESHOLD_MM = 50.0; // mm - adjust until sensor reliably detects ball
    public static long TUNING_DEBOUNCE_MS = 15; // ms - sensor debounce to avoid artifacts

    // Feeder servo positions and holder positions
    public static double TUNING_FEEDER_STOP_POS = 0; // neutral/stop position for feeder servos
    public static double TUNING_FEEDER_PUSH_POS = 0.05; // pushing position for feeders

    public static double TUNING_HOLDER_CLOSED_POS = 0.0; // blocks ball
    public static double TUNING_HOLDER_OPEN_POS = 0.05; // allows ball into shooter
    // ---------------------------------------------------------------------------------------

    private final double REAR_STACKER_SPEED = 0.8;
    private final double FRONT_STACKER_SPEED = 0.8;

    // distance sensor to detect ball between shooter and holder
    private DistanceSensor distanceSensor = null;

    // debounce for artifact detection / stable readings
    private boolean lastRawPresence = false;
    private boolean debouncedPresence = false;
    private boolean lastDebouncedPresence = false; // to detect falling edge
    private long lastChangeTimestamp = 0; // millis

    // legacy constructor: stacker motors + single holder (no feeders)
    public StackerSubsystem(DcMotor stackerLeftMotor, DcMotor stackerRightMotor, Servo holderServo) {
        this(stackerLeftMotor, stackerRightMotor, null, null, holderServo);
    }

    // new constructor: stacker motors + two feeder servos + one holder servo
    public StackerSubsystem(DcMotor stackerLeftMotor, DcMotor stackerRightMotor,
                            Servo feederLeftServo, Servo feederRightServo, Servo holderServo) {
        this.stackerLeftMotor = stackerLeftMotor;
        this.stackerRightMotor = stackerRightMotor;
        this.feederLeftServo = feederLeftServo;
        this.feederRightServo = feederRightServo;
        this.holderServo = holderServo;

        stackerLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        stackerRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        stackerLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        stackerRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        feederLeftServo.setDirection(Servo.Direction.FORWARD);
        feederRightServo.setDirection(Servo.Direction.FORWARD);
        holderServo.setDirection(Servo.Direction.FORWARD);

        // ensure holder closed and feeders stopped on init
        stopFeed();
        hold();
    }

    // set the distance sensor (call from OpMode init). It's optional; code will behave safely if null.
    public void setDistanceSensor(DistanceSensor sensor) {
        this.distanceSensor = sensor;
        // initialize debounce state from current reading if available
        if (sensor != null) {
            double d = sensor.getDistance(DistanceUnit.MM);
            boolean present = d < TUNING_PRESENCE_THRESHOLD_MM;
            lastRawPresence = present;
            debouncedPresence = present;
            lastDebouncedPresence = present;
            lastChangeTimestamp = System.currentTimeMillis();
        }
    }

    // Call this periodically (non-blocking). Returns true if a falling edge (ball left) was detected
    // since the last call. Uses DEBOUNCE_MS to avoid artifacts.
    public boolean updateSensor() {
        if (distanceSensor == null) return false;
        boolean rawPresent;
        try {
            double d = distanceSensor.getDistance(DistanceUnit.MM);
            rawPresent = d < TUNING_PRESENCE_THRESHOLD_MM;
        } catch (Exception e) {
            // if sensor read fails treat as not present
            rawPresent = false;
        }

        long now = System.currentTimeMillis();
        if (rawPresent != lastRawPresence) {
            // raw value changed: reset timer
            lastChangeTimestamp = now;
            lastRawPresence = rawPresent;
        }

        if ((now - lastChangeTimestamp) >= TUNING_DEBOUNCE_MS) {
            // stable for debounce interval => update debounced value if changed
            if (debouncedPresence != rawPresent) {
                lastDebouncedPresence = debouncedPresence;
                debouncedPresence = rawPresent;
                // falling edge: previously present, now not present
                if (lastDebouncedPresence && !debouncedPresence) {
                    return true;
                }
            }
        }
        return false;
    }

    // query the current debounced presence state
    public boolean isBallPresentDebounced() {
        return debouncedPresence;
    }

    // Reset the debounce/internal sensor state using the current sensor reading.
    // Call this before starting a shooting cycle so falling edges are detected fresh.
    public void resetDebounceFromCurrent() {
        long now = System.currentTimeMillis();
        if (distanceSensor == null) {
            lastRawPresence = false;
            debouncedPresence = false;
            lastDebouncedPresence = false;
            lastChangeTimestamp = now;
            return;
        }
        try {
            double d = distanceSensor.getDistance(DistanceUnit.MM);
            boolean present = d < TUNING_PRESENCE_THRESHOLD_MM;
            lastRawPresence = present;
            debouncedPresence = present;
            lastDebouncedPresence = present;
            lastChangeTimestamp = now;
        } catch (Exception e) {
            lastRawPresence = false;
            debouncedPresence = false;
            lastDebouncedPresence = false;
            lastChangeTimestamp = now;
        }
    }

    // run stacker to pull ball up
    public void pull() {
        stackerLeftMotor.setPower(FRONT_STACKER_SPEED);
        stackerRightMotor.setPower(REAR_STACKER_SPEED);
    }

    // reverse stacker
    public void repell() {
        stackerLeftMotor.setPower(-FRONT_STACKER_SPEED);
        stackerRightMotor.setPower(-REAR_STACKER_SPEED);
    }

    // holder: close (block) and open (allow)
    public void hold() {
        if (holderServo != null) holderServo.setPosition(TUNING_HOLDER_CLOSED_POS);
    }

    public void openHolder() {
        if (holderServo != null) holderServo.setPosition(TUNING_HOLDER_OPEN_POS);
    }

    // feeders: start a pushing motion (set to push position)
    public void feed() {
        if (feederLeftServo != null) feederLeftServo.setPosition(TUNING_FEEDER_PUSH_POS);
        if (feederRightServo != null) feederRightServo.setPosition(TUNING_FEEDER_PUSH_POS);
    }

    // stop feeder servos (return to neutral)
    public void stopFeed() {
        if (feederLeftServo != null) feederLeftServo.setPosition(TUNING_FEEDER_STOP_POS);
        if (feederRightServo != null) feederRightServo.setPosition(TUNING_FEEDER_STOP_POS);
    }

    // stop stacker motors
    public void stop() {
        stackerLeftMotor.setPower(0);
        stackerRightMotor.setPower(0);
    }
}

