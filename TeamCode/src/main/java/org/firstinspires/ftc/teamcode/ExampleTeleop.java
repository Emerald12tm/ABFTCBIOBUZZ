package org.firstinspires.ftc.teamcode;

import com.pedropathing.controllers.PIDController;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/*
 * Controls (all on gamepad1)
 *   left stick       drive / strafe
 *   right stick x    turn
 *   (release sticks) robot holds its position
 *   X (hold)         auto align 1 - turn to the heading the robot had when TeleOp started
 *   B (hold)         auto align 2 - turn to auto align 1 + 180 degrees
 *   right trigger    intake in
 *   left trigger     intake out + windmill reverse (unjam)
 *   A                toggle shooter on/off
 *   right bumper     hold to spin windmill and feed balls (only while shooter is at speed)
 */
@TeleOp(name = "Example Teleop")
public class ExampleTeleop extends OpMode {

    // Hardware config names - change these to match your robot configuration
    public static String INTAKE_MOTOR = "intake";
    public static String INTAKE_SERVO_LEFT = "intake_left_servo";
    public static String INTAKE_SERVO_RIGHT = "intake_right_servo";
    public static String INDEXER_SERVO = "windmillServo";

    // Intake
    public static double INTAKE_POWER = 1.0;
    public static double INTAKE_MOTOR_SPEED = 1; // fraction of INTAKE_POWER for the motor only

    // Indexer (continuous rotation servo) - change the direction if it spins the wrong way.
    // Also used by the autos and ShooterTuner.
    public static double INDEXER_POWER = 1.0;
    public static DcMotorSimple.Direction INDEXER_DIRECTION = DcMotorSimple.Direction.REVERSE;

    // Heading lock PID (turn power per radian of error)
    public static double HEADING_kP = 1.0;
    public static double HEADING_kI = 0.0;
    public static double HEADING_kD = 0.05;

    // Pose used when no autonomous ran before this TeleOp
    public static Pose DEFAULT_START_POSE = new Pose(0, 0, 0);

    private Follower follower;
    private PIDController headingPid;
    private double alignHeading1;
    private double alignHeading2;

    private DcMotor intakeMotor;
    private CRServo intakeServoLeft;
    private CRServo intakeServoRight;
    private CRServo indexerServo;
    private Shooter shooter;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        headingPid = new PIDController(HEADING_kP, HEADING_kI, HEADING_kD);

        intakeMotor = hardwareMap.get(DcMotor.class, INTAKE_MOTOR);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeServoLeft = hardwareMap.get(CRServo.class, INTAKE_SERVO_LEFT);
        intakeServoRight = hardwareMap.get(CRServo.class, INTAKE_SERVO_RIGHT);
        // Reverse one side so the same power spins them in opposite directions
        intakeServoLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        indexerServo = hardwareMap.get(CRServo.class, INDEXER_SERVO);
        indexerServo.setDirection(INDEXER_DIRECTION);
        indexerServo.setPower(0);

        shooter = new Shooter(hardwareMap);
    }

    @Override
    public void start() {
        // Continue from where autonomous left off, if it ran
        Pose startPose = OpModeStorage.autonomousEndPose != null
                ? OpModeStorage.autonomousEndPose
                : DEFAULT_START_POSE;
        follower.setPose(startPose);
        follower.update();

        alignHeading1 = startPose.heading();
        alignHeading2 = alignHeading1 + Math.PI;
    }

    @Override
    public void loop() {
        updateDrive();
        follower.update();

        updateIntake();
        updateShooter();
        updateIndexer();

        telemetry.update();
    }

    private void updateDrive() {
        DrivePowers powers = new DrivePowers(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        );

        String driveMode;
        if (gamepad1.x || gamepad1.b) {
            double targetHeading = gamepad1.x ? alignHeading1 : alignHeading2;
            driveMode = gamepad1.x ? "AUTO ALIGN 1" : "AUTO ALIGN 2";

            headingPid.kP = HEADING_kP;
            headingPid.kI = HEADING_kI;
            headingPid.kD = HEADING_kD;
            // Keeps forward/strafe from the stick, replaces turn with the heading PID output
            follower.manual(ManualDrive.headingLock(follower, headingPid, powers, targetHeading));
        } else {
            headingPid.reset();
            // Drives normally, and holds the current pose once the sticks are released
            ManualDrive.driveOrHold(follower, powers);
            driveMode = follower.mode().toString();
        }

        Pose pose = follower.pose();
        telemetry.addData("Drive Mode", driveMode);
        telemetry.addData("X", pose.x());
        telemetry.addData("Y", pose.y());
        telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading()));
    }

    private void updateIntake() {
        double power = 0;
        if (gamepad1.right_trigger > 0.1) {
            power = INTAKE_POWER;
        } else if (gamepad1.left_trigger > 0.1) {
            power = -INTAKE_POWER;
        }

        intakeMotor.setPower(power * INTAKE_MOTOR_SPEED);
        intakeServoLeft.setPower(power);
        intakeServoRight.setPower(power);
    }

    private void updateShooter() {
        if (gamepad1.aWasPressed()) {
            shooter.toggle();
        }
        shooter.setTarget(Shooter.defaultTarget());
        shooter.update();

        telemetry.addData("Shooter On", shooter.isOn());
        telemetry.addData("Shooter Target (tps)", shooter.target());
        telemetry.addData("Shooter Velocity (tps)", shooter.velocity());
        telemetry.addData("Shooter Power", shooter.power());
        telemetry.addData("Shooter At Speed", shooter.atSpeed());
    }

    private void updateIndexer() {
        String state;
        double power = 0;
        if (gamepad1.left_trigger > 0.1) {
            power = -INDEXER_POWER;
            state = "REVERSE";
        } else if (gamepad1.right_bumper && shooter.atSpeed()) {
            power = INDEXER_POWER;
            state = "FEED";
        } else if (gamepad1.right_bumper) {
            state = shooter.isOn() ? "WAITING - shooter not at speed" : "WAITING - shooter is off (press A)";
        } else {
            state = "STOPPED";
        }
        indexerServo.setPower(power);

        telemetry.addData("Windmill", state);
    }
}
