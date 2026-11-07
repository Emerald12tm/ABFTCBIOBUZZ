package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;

/*
 * Static fields here survive between OpModes, so autonomous can hand values to TeleOp.
 *
 * In your autonomous, save the final pose when it ends:
 *
 *     @Override
 *     public void stop() {
 *         OpModeStorage.autonomousEndPose = follower.pose();
 *     }
 */
public class OpModeStorage {
    /** Pose the robot ended autonomous at, or null if no autonomous has run since the app started. */
    public static Pose autonomousEndPose = null;
}
