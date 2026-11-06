package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Template for a Pedro autonomous. To make a new one, copy this file, change the class name and the
 * name in @Autonomous, then edit steps 1-3 below.
 */
@SuppressWarnings("unused") // discovered via reflection by the FTC Driver Station, not referenced directly
@Autonomous(name = "First Auto", group = "COMP")
public class FirstAuto extends LinearOpMode {

    private final PoseFactory poses = PoseFactory.degrees();

    // STEP 1: poses from the visualizer, as (x inches, y inches, heading degrees).
    // startPose must match where the robot is really placed on the field.
    private final Pose startPose = poses.of(78.7053, 8, 90);
    private final Pose path1 = poses.of(80.6655, 26.3625, 180);

    // STEP 2: one path per move, in the order the robot should drive them.
    //   Straight line:  line(a, b).linear(a, b)
    //   Curve:          curve(a, controlPoint, b).linear(a, b)
    private Path[] buildPaths() {
        return new Path[] {
                line(startPose, path1).linear(startPose, path1),
        };
    }

    @Override
    public void runOpMode() {
        Follower follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();

        Path[] paths = buildPaths();

        telemetry.addData("Auto", "Ready");
        telemetry.update();

        waitForStart();

        for (int i = 0; i < paths.length && opModeIsActive(); i++) {
            follower.follow(paths[i]);

            while (opModeIsActive() && !follower.atParametricEnd()) {
                follower.update();
                telemetry.addData("Path", "%d of %d", i + 1, paths.length);
                telemetry.addData("X", "%.1f in", follower.pose().x());
                telemetry.addData("Y", "%.1f in", follower.pose().y());
                telemetry.addData("Heading", "%.1f deg", Math.toDegrees(follower.pose().heading()));
                telemetry.update();
            }

            // STEP 3: anything the robot should do after path i goes here, like shooting.
        }

        follower.stop();
    }
}
