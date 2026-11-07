package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Shooter PIDF tuner (gamepad1)
 *   Y                measure max speed + kF (spins at 90% power for a few seconds)
 *   A                shooter on/off at the target speed
 *   dpad left/right  pick which value to change
 *   dpad up/down     increase / decrease the picked value
 *   bumpers          make the step bigger (right) / smaller (left)
 *   right trigger    run intake + windmill to fire balls and test recovery
 *
 * Gains changed here last until the robot app restarts. Copy the final values
 * shown on telemetry into Shooter.java.
 */
@TeleOp(name = "Shooter Tuner", group = "Tuning")
public class ShooterTuner extends OpMode {

    // Below 1.0 so voltage compensation never clips during the measurement
    private static final double MEASURE_POWER = 0.9;
    private static final double MEASURE_SPINUP_SEC = 2.5;
    private static final double MEASURE_SAMPLE_SEC = 1.5;

    private static final String[] NAMES = {"kP", "kI", "kD", "kF", "Target"};
    private final double[] steps = {0.0005, 0.0001, 0.00001, 0.00001, 50};
    private int selected = 0;

    private Shooter shooter;
    private DcMotor intakeMotor;
    private CRServo intakeServoLeft;
    private CRServo intakeServoRight;
    private CRServo indexerServo;

    private double target;

    // kF measurement
    private boolean measuring = false;
    private final ElapsedTime measureTimer = new ElapsedTime();
    private double velocitySum = 0;
    private int velocitySamples = 0;
    private String measureResult = "not run (press Y)";

    // Shot recovery tracking
    private boolean wasAtSpeed = false;
    private boolean recovering = false;
    private final ElapsedTime recoveryTimer = new ElapsedTime();
    private double minVelocityDuringDip = 0;
    private String lastRecovery = "-";

    @Override
    public void init() {
        shooter = new Shooter(hardwareMap);
        target = Shooter.defaultTarget();

        intakeMotor = hardwareMap.get(DcMotor.class, ExampleTeleop.INTAKE_MOTOR);
        intakeServoLeft = hardwareMap.get(CRServo.class, ExampleTeleop.INTAKE_SERVO_LEFT);
        intakeServoRight = hardwareMap.get(CRServo.class, ExampleTeleop.INTAKE_SERVO_RIGHT);
        intakeServoLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        indexerServo = hardwareMap.get(CRServo.class, ExampleTeleop.INDEXER_SERVO);
        indexerServo.setDirection(ExampleTeleop.INDEXER_DIRECTION);
    }

    @Override
    public void loop() {
        handleInput();

        if (measuring) {
            updateMeasurement();
        } else {
            shooter.setTarget(target);
        }
        shooter.update();
        trackRecovery();

        // Fire balls to test how fast the shooter recovers
        boolean feeding = gamepad1.right_trigger > 0.1 && shooter.atSpeed();
        double feedPower = gamepad1.right_trigger > 0.1 ? ExampleTeleop.INTAKE_POWER : 0;
        intakeMotor.setPower(feedPower * ExampleTeleop.INTAKE_MOTOR_SPEED);
        intakeServoLeft.setPower(feedPower);
        intakeServoRight.setPower(feedPower);
        indexerServo.setPower(feeding ? ExampleTeleop.INDEXER_POWER : 0);

        showTelemetry();
    }

    private void handleInput() {
        if (gamepad1.yWasPressed() && !measuring) {
            measuring = true;
            measureTimer.reset();
            velocitySum = 0;
            velocitySamples = 0;
            shooter.runOpenLoop(MEASURE_POWER);
        }
        if (gamepad1.aWasPressed()) {
            measuring = false;
            shooter.toggle();
        }

        if (gamepad1.dpadRightWasPressed()) selected = (selected + 1) % NAMES.length;
        if (gamepad1.dpadLeftWasPressed()) selected = (selected + NAMES.length - 1) % NAMES.length;
        if (gamepad1.rightBumperWasPressed()) steps[selected] *= 10;
        if (gamepad1.leftBumperWasPressed()) steps[selected] /= 10;
        if (gamepad1.dpadUpWasPressed()) adjust(steps[selected]);
        if (gamepad1.dpadDownWasPressed()) adjust(-steps[selected]);
    }

    private void adjust(double delta) {
        switch (selected) {
            case 0: Shooter.kP = Math.max(0, Shooter.kP + delta); break;
            case 1: Shooter.kI = Math.max(0, Shooter.kI + delta); break;
            case 2: Shooter.kD = Math.max(0, Shooter.kD + delta); break;
            case 3: Shooter.kF = Math.max(0, Shooter.kF + delta); break;
            case 4: target = Math.max(0, target + delta); break;
        }
    }

    private void updateMeasurement() {
        double t = measureTimer.seconds();
        if (t > MEASURE_SPINUP_SEC) {
            velocitySum += shooter.velocity();
            velocitySamples++;
        }
        if (t > MEASURE_SPINUP_SEC + MEASURE_SAMPLE_SEC) {
            measuring = false;
            shooter.setOn(false);

            double avgVelocity = velocitySum / Math.max(1, velocitySamples);
            if (avgVelocity < 100) {
                measureResult = "FAILED - velocity ~0. Check encoder cable / motor direction";
                return;
            }
            Shooter.MAX_TICKS_PER_SEC = avgVelocity / MEASURE_POWER;
            Shooter.kF = MEASURE_POWER / avgVelocity;
            target = Shooter.defaultTarget();
            measureResult = String.format("max %.0f tps, kF %.6f", Shooter.MAX_TICKS_PER_SEC, Shooter.kF);
        }
    }

    private void trackRecovery() {
        boolean atSpeed = shooter.atSpeed();
        if (wasAtSpeed && !atSpeed && shooter.isOn()) {
            // Speed just dropped (a ball went through)
            recovering = true;
            recoveryTimer.reset();
            minVelocityDuringDip = shooter.velocity();
        }
        if (recovering) {
            minVelocityDuringDip = Math.min(minVelocityDuringDip, shooter.velocity());
            if (atSpeed) {
                recovering = false;
                lastRecovery = String.format("%.0f ms, dipped %.0f tps",
                        recoveryTimer.milliseconds(), target - minVelocityDuringDip);
            } else if (!shooter.isOn()) {
                recovering = false;
            }
        }
        wasAtSpeed = atSpeed;
    }

    private void showTelemetry() {
        telemetry.addData("Mode", measuring ? "MEASURING kF..." : shooter.isOn() ? "PID ON" : "OFF");
        telemetry.addData("Target (tps)", "%.0f", shooter.target());
        telemetry.addData("Velocity (tps)", "%.0f", shooter.velocity());
        telemetry.addData("Error (tps)", "%.0f", shooter.target() - shooter.velocity());
        telemetry.addData("Power", "%.3f", shooter.power());
        telemetry.addData("Battery (V)", "%.2f", shooter.voltage());
        telemetry.addData("At Speed", shooter.atSpeed());
        telemetry.addData("Last shot recovery", lastRecovery);
        telemetry.addData("kF measurement", measureResult);
        telemetry.addLine();
        telemetry.addData("Editing", "%s  (step %s)", NAMES[selected], steps[selected]);
        telemetry.addLine("--- copy these into Shooter.java ---");
        telemetry.addData("MAX_TICKS_PER_SEC", "%.0f", Shooter.MAX_TICKS_PER_SEC);
        telemetry.addData("kP", "%.6f", Shooter.kP);
        telemetry.addData("kI", "%.6f", Shooter.kI);
        telemetry.addData("kD", "%.6f", Shooter.kD);
        telemetry.addData("kF", "%.6f", Shooter.kF);
        telemetry.update();
    }
}
