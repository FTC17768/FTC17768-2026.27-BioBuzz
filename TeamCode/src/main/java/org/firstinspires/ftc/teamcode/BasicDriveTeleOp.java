package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MechanismSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;

@SuppressWarnings("unused") // discovered via reflection by the FTC Driver Station, not referenced directly
@TeleOp(name="Basic Drive TeleOp", group="COMP")
public class BasicDriveTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry.addData("Robot", "Initializing Subsystems...");
        telemetry.update();

        // Instantiate subsystems
        DriveSubsystem drive = new DriveSubsystem(hardwareMap);
        OdometrySubsystem odometry = new OdometrySubsystem(hardwareMap);
        MechanismSubsystem mechanisms = new MechanismSubsystem(hardwareMap);

        // One-time sanity check that we're talking to the right sensor with the offsets we expect.
        telemetry.addData("Pinpoint Device", "ID: %d, Firmware: %d", odometry.getDeviceId(), odometry.getDeviceVersion());
        telemetry.addData("Pinpoint Offsets", "X: %.1fmm, Y: %.1fmm, YawScalar: %.4f",
                odometry.getXOffsetMm(), odometry.getYOffsetMm(), odometry.getYawScalar());

        telemetry.addData("Robot", "Ready!");
        telemetry.update();

        waitForStart();

        double speed = 0.5;
        double speed_slow = 0.25;
        double speed_normal = 0.5;
        double speed_fast = 1.0;

        boolean lastResetButtonState = false;
        boolean lastRecalibrateButtonState = false;
        boolean shooterOn = false;
        boolean lastShooterButtonState = false;

        while (opModeIsActive()) {

            // Re-zero odometry on a fresh press of "back". Robot should be stationary when pressed.
            boolean resetButtonState = gamepad1.back;
            if (resetButtonState && !lastResetButtonState) {
                odometry.resetPosAndIMU();
            }
            lastResetButtonState = resetButtonState;

            // Recalibrate the gyro only (keeps tracked position) on a fresh press of "y".
            // Useful mid-match if heading appears to be drifting, without losing field position.
            boolean recalibrateButtonState = gamepad1.y;
            if (recalibrateButtonState && !lastRecalibrateButtonState) {
                odometry.recalibrateIMU();
            }
            lastRecalibrateButtonState = recalibrateButtonState;

            // Read Pinpoint odometry
            odometry.update();
            Pose2D pose = odometry.getPosition();

            // Controller speed toggles
            if (gamepad1.b) {
                speed = speed_slow;
            } else if (gamepad1.x) {
                speed = speed_normal;
            } else if (gamepad1.a) {
                speed = speed_fast;
            }

            // Controller axis inputs
            float forward = gamepad1.left_stick_y;
            float strafe = gamepad1.left_stick_x;
            float steer = gamepad1.right_stick_x;

            // Command the Drive Subsystem
            drive.drive(forward, strafe, steer, speed);

            // Intake: hold right bumper to run in, left bumper to reverse
            if (gamepad1.right_trigger > 0.8) {
                mechanisms.intakeIn();
            } else if (gamepad1.left_trigger > 0.8) {
                mechanisms.intakeOut();
            } else {
                mechanisms.intakeStop();
            }

            // Shooter: a fresh press of the right trigger toggles it on or off
            boolean shooterButtonState = gamepad2.left_trigger > 0.85;
            if (shooterButtonState && !lastShooterButtonState) {
                shooterOn = !shooterOn;
            }
            lastShooterButtonState = shooterButtonState;
            mechanisms.setShooterOn(shooterOn);

            // Flapper: feeds the shooter while the left trigger is held, once the shooter is ready
            mechanisms.setFlapperOn(gamepad2.right_trigger > 0.85);

            // Telemetry feedback
            telemetry.addData("Shooter", !shooterOn ? "OFF" : (mechanisms.isShooterReady() ? "READY" : "SPINNING UP"));
            telemetry.addData("Shooter Speed", "%.0f ticks/s", mechanisms.getShooterVelocity());
            telemetry.addData("Speed Mode", speed == speed_slow ? "SLOW" : (speed == speed_fast ? "FAST" : "NORMAL"));
            telemetry.addData("Drive", "Forward: %.2f, Strafe: %.2f, Steer: %.2f", forward, strafe, steer);

            // Odometry telemetry
            telemetry.addData("Odometry Status", odometry.getStatus());
            telemetry.addData("Odometry Pose", "X: %.1fmm, Y: %.1fmm, H: %.1f°",
                    pose.getX(DistanceUnit.MM),
                    pose.getY(DistanceUnit.MM),
                    odometry.getHeadingDegrees());
            telemetry.addData("Odometry Pose (in)", "X: %.2fin, Y: %.2fin",
                    pose.getX(DistanceUnit.INCH),
                    pose.getY(DistanceUnit.INCH),
                    odometry.getHeadingDegrees());
            telemetry.addData("Odometry Velocity", "X: %.1fmm/s, Y: %.1fmm/s, Turn: %.1f°/s",
                    odometry.getVelXMmPerSec(),
                    odometry.getVelYMmPerSec(),
                    odometry.getHeadingVelocityDegPerSec());
            telemetry.addData("Odometry Encoders", "X: %d ticks, Y: %d ticks",
                    odometry.getEncoderXTicks(),
                    odometry.getEncoderYTicks());
            telemetry.addData("Odometry Loop", "%d us (%.0f Hz)",
                    odometry.getLoopTimeMicros(),
                    odometry.getFrequencyHz());
            telemetry.addData("Odometry Controls", "BACK: re-zero position+gyro | Y: recalibrate gyro only");

            telemetry.update();
        }
    }
}