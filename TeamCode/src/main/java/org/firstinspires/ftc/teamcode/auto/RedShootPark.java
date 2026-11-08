package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.PathFile;
import org.firstinspires.ftc.teamcode.subsystems.MechanismSubsystem;

/**
 * Template for a Pedro autonomous. To make a new one, copy this file, change the class name and the
 * name in @Autonomous, then edit steps 1-3 below.
 */
@SuppressWarnings("unused") // discovered via reflection by the FTC Driver Station, not referenced directly
@Autonomous(name = "Red Shoot and Park", group = "COMP")
public class RedShootPark extends LinearOpMode {

    private Follower follower;
    private MechanismSubsystem mechanisms;

    @Override
    public void runOpMode() {
        follower = Constants.create(hardwareMap);
        mechanisms = new MechanismSubsystem(hardwareMap);

        // STEP 1: the file saved from the visualizer, in TeamCode/src/main/assets/paths/.
        // Place the robot at the start point drawn in that file.
        PathFile paths = PathFile.load(hardwareMap, "ParkRed");
        follower.setPose(paths.startPose());
        follower.update();

        // STEP 2: get each path by its name in the visualizer.
        Path Park = paths.get("Park");

        telemetry.addData("Auto", "Ready");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            follower.update();
            telemetry.addData("Pose X", "%.1f", follower.pose().x());
            telemetry.addData("Pose Y", "%.1f", follower.pose().y());
            telemetry.addData("Heading", "%.1f deg", Math.toDegrees(follower.pose().heading()));
            telemetry.update();
        }

        // STEP 3: the order of actions. Other things you can add here:
             mechanisms.setShooterOn(true);                     //spin up, keeps running while driving
             shoot(10.0);                                        //feed for 2 seconds while READY
        //   mechanisms.intakeIn();  /  mechanisms.intakeStop();
        //   driveThen(firstPath, 0.5, mechanisms::intakeIn);   do something partway along a path
        mechanisms.setShooterOn(false);
        drive(Park);
        mechanisms.intakeStop();
        follower.stop();
    }

    /** Drives a path and waits until it's done. */
    private void drive(Path path) {
        driveThen(path, 2.0, () -> { }); // 2.0 is past the end of a path, so the action never runs
    }

    /** Drives a path, running the action once when the path reaches atFraction (0 is the start, 1 the end). */
    private void driveThen(Path path, double atFraction, Runnable action) {
        boolean done = false;
        follower.follow(path);

        while (opModeIsActive() && !follower.atParametricEnd()) {
            follower.update();

            double completion = follower.parametricCompletion();
            if (!done && completion >= atFraction) {
                action.run();
                done = true;
            }

            telemetry.addData("Path done", "%.2f", completion);
            telemetry.addData("X", "%.1f in", follower.pose().x());
            telemetry.addData("Y", "%.1f in", follower.pose().y());
            telemetry.addData("Heading", "%.1f deg", Math.toDegrees(follower.pose().heading()));
            telemetry.update();
        }
    }

    /** Holds position and feeds for this many seconds. The flapper only runs while the shooter is READY. */
    private void shoot(double seconds) {
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && timer.seconds() < seconds) {
            follower.update();
            mechanisms.setFlapperOn(true);
        }

        mechanisms.setFlapperOn(false);
    }
}
