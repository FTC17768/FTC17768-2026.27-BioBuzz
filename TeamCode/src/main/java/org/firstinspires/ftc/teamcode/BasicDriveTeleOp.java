package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@SuppressWarnings("unused") // discovered via reflection by the FTC Driver Station, not referenced directly
@TeleOp(name="Basic Drive TeleOp", group="COMP")
public class BasicDriveTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry.addData("Robot", "Initializing Subsystems...");
        telemetry.update();

        // Instantiate subsystems
        DriveSubsystem drive = new DriveSubsystem(hardwareMap);
        VisionSubsystem vision = new VisionSubsystem(hardwareMap);
        OdometrySubsystem odometry = new OdometrySubsystem(hardwareMap);

        // One-time sanity check that we're talking to the right sensor with the offsets we expect.
        telemetry.addData("Pinpoint Device", "ID: %d, Firmware: %d", odometry.getDeviceId(), odometry.getDeviceVersion());
        telemetry.addData("Pinpoint Offsets", "X: %.1fmm, Y: %.1fmm, YawScalar: %.4f",
                odometry.getXOffsetMm(), odometry.getYOffsetMm(), odometry.getYawScalar());

        telemetry.addData("Robot", "Ready!");
        telemetry.update();

        waitForStart();
        
        // Start Limelight polling
        vision.start();

        double speed = 0.6;
        double speed_slow = 0.4;
        double speed_normal = 0.6;
        double speed_fast = 0.8;

        boolean lastResetButtonState = false;
        boolean lastRecalibrateButtonState = false;

        while (opModeIsActive()) {
            // Read Limelight
            LLResult llResult = vision.getLatestResult();

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
            if (gamepad1.x) {
                speed = speed_slow;
            } else if (gamepad1.a) {
                speed = speed_normal;
            } else if (gamepad1.b) {
                speed = speed_fast;
            }

            // Controller axis inputs
            float forward = gamepad1.left_stick_y;
            float strafe = gamepad1.left_stick_x;
            float steer = gamepad1.right_stick_x;

            // Command the Drive Subsystem
            drive.drive(forward, strafe, steer, speed);

            // Telemetry feedback
            telemetry.addData("Speed Mode", speed == speed_slow ? "SLOW" : (speed == speed_fast ? "FAST" : "NORMAL"));
            telemetry.addData("Drive", "Forward: %.2f, Strafe: %.2f, Steer: %.2f", forward, strafe, steer);
            
            // Basic Limelight telemetry
            if (llResult != null && llResult.isValid()) {
                telemetry.addData("Limelight", "Target Visible! tx: %.2f, ty: %.2f", llResult.getTx(), llResult.getTy());
            } else {
                telemetry.addData("Limelight", "No Target");
            }

            // Odometry telemetry
            telemetry.addData("Odometry Status", odometry.getStatus());
            telemetry.addData("Odometry Pose", "X: %.1fmm, Y: %.1fmm, H: %.1f°",
                    pose.getX(DistanceUnit.MM),
                    pose.getY(DistanceUnit.MM),
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
        
        // Clean up
        vision.stop();
    }
}