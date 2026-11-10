package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/*
 * Shared autonomous routine used by RedSideAuto and BlueSideAuto.
 *
 *   1. Shoot from the starting spot, shooter off
 *   2. Drive to the pickup spot with the intake running (path 1)
 *   3. Drive to the shooting spot, spin up the shooter, fire (path 2)
 *   4. Turn everything off and drive to the landing zone (path 3)
 */
public abstract class SideAuto extends LinearOpMode {

    // How long to feed balls at each shooting spot
    public static double SHOOT_TIME_MS = 3000;
    // Give up waiting for the shooter to reach speed after this long and shoot anyway
    public static double SPINUP_TIMEOUT_MS = 2500;
    // Keep the intake running this long after reaching the pickup spot
    public static double INTAKE_FINISH_MS = 500;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose point1 = poseFactory.of(11.5514, 8.4472, 90);
    private final Pose point1Control1 = poseFactory.of(9.7716, 47.1133, 0);
    private final Pose point2Start = poseFactory.of(11.5514, 8.4472, 90);
    private final Pose point2 = poseFactory.of(59.499, 133.5507, 270);
    private final Pose point2Control1 = poseFactory.of(13.0538, 63.2034, 0);
    private final Pose point3Start = poseFactory.of(59.499, 133.5507, 270);
    private final Pose point3 = poseFactory.of(11.9968, 94.2523, 270);
    private final Pose point3Control1 = poseFactory.of(42.8224, 93.8085, 0);

    private Follower follower;
    private Shooter shooter;
    private DcMotor intakeMotor;
    private CRServo intakeServoLeft;
    private CRServo intakeServoRight;
    private CRServo indexerServo;

    private String step = "Waiting for start";

    public Command autoRoutine() {
        return sequential(
                // Shoot from the starting spot
                shoot(),

                // Pick up balls
                setStep("Intaking"),
                instant(() -> setIntake(ExampleTeleop.INTAKE_POWER)),
                follow(follower, path1()),
                waitMs(INTAKE_FINISH_MS),
                instant(() -> setIntake(0)),

                // Shooting spot
                setStep("Driving to shooting spot"),
                follow(follower, path2()),
                shoot(),

                // Landing zone
                setStep("Driving to landing zone"),
                instant(this::stopAll),
                follow(follower, path3()),
                setStep("Done")
        );
    }

    /** Spins up the shooter, feeds balls for SHOOT_TIME_MS, then turns everything off. */
    private Command shoot() {
        return sequential(
                setStep("Spinning up shooter"),
                instant(() -> shooter.setOn(true)),
                race(waitUntil(shooter::atSpeed), waitMs(SPINUP_TIMEOUT_MS)),
                setStep("Shooting"),
                // Only feeds while the shooter is at speed, so it pauses to recover between balls
                race(waitMs(SHOOT_TIME_MS), infinite(this::feedWhenReady)),
                instant(this::stopAll)
        );
    }

    private Command setStep(String name) {
        return instant(() -> step = name);
    }

    private void feedWhenReady() {
        boolean ready = shooter.atSpeed();
        setIntake(ready ? ExampleTeleop.INTAKE_POWER : 0);
        indexerServo.setPower(ready ? ExampleTeleop.INDEXER_POWER : 0);
    }

    private void setIntake(double power) {
        intakeMotor.setPower(power * ExampleTeleop.INTAKE_MOTOR_SPEED);
        intakeServoLeft.setPower(power);
        intakeServoRight.setPower(power);
    }

    private void stopAll() {
        setIntake(0);
        indexerServo.setPower(0);
        shooter.setOn(false);
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        intakeMotor = hardwareMap.get(DcMotor.class, ExampleTeleop.INTAKE_MOTOR);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeServoLeft = hardwareMap.get(CRServo.class, ExampleTeleop.INTAKE_SERVO_LEFT);
        intakeServoRight = hardwareMap.get(CRServo.class, ExampleTeleop.INTAKE_SERVO_RIGHT);
        // Same direction setup as TeleOp
        intakeServoLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        indexerServo = hardwareMap.get(CRServo.class, ExampleTeleop.INDEXER_SERVO);
        indexerServo.setDirection(ExampleTeleop.INDEXER_DIRECTION);
        shooter = new Shooter(hardwareMap);
        shooter.setTarget(Shooter.defaultTarget());

        waitForStart();
        if (isStopRequested()) return;

        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();
            shooter.update();

            telemetry.addData("Step", step);
            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading (deg)", Math.toDegrees(follower.pose().heading()));
            telemetry.addData("Shooter velocity (tps)", shooter.velocity());
            telemetry.addData("Shooter at speed", shooter.atSpeed());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }

        // Let TeleOp continue from where auto ended
        OpModeStorage.autonomousEndPose = follower.pose();
    }

    public Path path1() {
        return Paths.curve(start, point1Control1, point1).linear(start, point1);
    }

    public Path path2() {
        return Paths.curve(point2Start, point2Control1, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.curve(point3Start, point3Control1, point3).linear(point3Start, point3);
    }
}
