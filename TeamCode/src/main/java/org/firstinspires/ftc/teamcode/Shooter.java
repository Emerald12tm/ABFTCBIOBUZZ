package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Flywheel shooter with a velocity PIDF controller and battery voltage compensation.
 * Shared by ExampleTeleop and ShooterTuner so both use the same gains.
 */
public class Shooter {

    public static String MOTOR_NAME = "launcher";

    // Max free speed in ticks/sec. Run ShooterTuner and press Y to measure your real value.
    // Default is a goBILDA 6000 RPM motor (28 ticks/rev * 6000 / 60).
    public static double MAX_TICKS_PER_SEC = 2800;
    public static double SPEED = 0.45; // fraction of max speed used in TeleOp and auto
    public static double VELOCITY_TOLERANCE = 60; // ticks/sec

    // PIDF gains. kF = power per tick/sec at 12V; ShooterTuner can measure it for you.
    public static double kP = 0.002;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 1.0 / MAX_TICKS_PER_SEC;

    // Output is scaled so the shooter behaves the same on a full or tired battery
    public static double NOMINAL_VOLTAGE = 12.0;

    // Brake the flywheel to a quick stop when the shooter is turned off.
    // While running it floats, so the PID never fights a braking motor.
    public static boolean BRAKE_WHEN_OFF = true;

    private final DcMotorEx motor;
    private final VoltageSensor battery;

    private boolean on = false;
    private double target = 0;
    private double openLoopPower = Double.NaN;

    private double velocity = 0;
    private double voltage = NOMINAL_VOLTAGE;
    private double power = 0;

    private double integral = 0;
    private double lastError = 0;
    private boolean firstRun = true;
    private final ElapsedTime timer = new ElapsedTime();

    public Shooter(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // We run our own PID, so don't use the hub's built-in velocity control
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        battery = hardwareMap.voltageSensor.iterator().next();
        target = defaultTarget();
        applyZeroPowerBehavior();
    }

    private void applyZeroPowerBehavior() {
        motor.setZeroPowerBehavior(!on && BRAKE_WHEN_OFF
                ? DcMotor.ZeroPowerBehavior.BRAKE
                : DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public static double defaultTarget() {
        return SPEED * MAX_TICKS_PER_SEC;
    }

    public void setOn(boolean on) {
        boolean changed = on != this.on;
        this.on = on;
        openLoopPower = Double.NaN;
        if (changed) {
            resetPid();
            applyZeroPowerBehavior();
        }
    }

    public void toggle() {
        setOn(!on);
    }

    public boolean isOn() {
        return on;
    }

    public void setTarget(double ticksPerSec) {
        target = ticksPerSec;
    }

    /** Bypasses the PID and runs a fixed (voltage compensated) power. Used to measure kF. */
    public void runOpenLoop(double power) {
        on = true;
        openLoopPower = power;
        applyZeroPowerBehavior();
    }

    /** Call once per loop. */
    public void update() {
        velocity = motor.getVelocity();
        voltage = battery.getVoltage();

        double output;
        if (!on) {
            output = 0;
        } else if (!Double.isNaN(openLoopPower)) {
            output = openLoopPower;
        } else {
            output = calculatePid();
        }

        // Scale for battery voltage, then clip - shooter only spins forward
        double compensation = voltage > 1 ? NOMINAL_VOLTAGE / voltage : 1;
        power = Range.clip(output * compensation, 0, 1);
        motor.setPower(power);
    }

    private double calculatePid() {
        double error = target - velocity;
        double dt = timer.seconds();
        timer.reset();

        double derivative = 0;
        if (firstRun || dt <= 0) {
            firstRun = false;
        } else {
            integral += error * dt;
            // Prevent integral windup
            if (kI != 0) {
                integral = Range.clip(integral, -0.3 / kI, 0.3 / kI);
            }
            derivative = (error - lastError) / dt;
        }
        lastError = error;

        return kF * target + kP * error + kI * integral + kD * derivative;
    }

    private void resetPid() {
        integral = 0;
        lastError = 0;
        firstRun = true;
    }

    public boolean atSpeed() {
        return on && Double.isNaN(openLoopPower) && Math.abs(target - velocity) < VELOCITY_TOLERANCE;
    }

    public double velocity() {
        return velocity;
    }

    public double target() {
        return target;
    }

    public double power() {
        return power;
    }

    public double voltage() {
        return voltage;
    }
}
