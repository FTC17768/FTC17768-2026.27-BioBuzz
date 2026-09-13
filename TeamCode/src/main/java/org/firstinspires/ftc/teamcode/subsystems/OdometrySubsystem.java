package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

public class OdometrySubsystem {

    // TODO: must match the Pinpoint's name in the robot's hardware configuration
    private static final String DEVICE_NAME = "pinpoint";

    // TODO: measure once the odometry pods are physically mounted and update these.
    // Left of center is positive X offset, forward of center is positive Y offset.
    private static final double X_POD_OFFSET_MM = 0.0;
    private static final double Y_POD_OFFSET_MM = 0.0;

    // TODO: confirm pod hardware and encoder wiring direction on the robot
    private static final GoBildaPinpointDriver.GoBildaOdometryPods POD_TYPE =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    private static final GoBildaPinpointDriver.EncoderDirection X_ENCODER_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    private static final GoBildaPinpointDriver.EncoderDirection Y_ENCODER_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    private final GoBildaPinpointDriver pinpoint;

    // Read once at init rather than every loop; these don't change during a match
    // and each read is its own I2C transaction (the driver's javadoc warns against
    // polling getXOffset()/getYOffset() every loop).
    private final int deviceId;
    private final int deviceVersion;
    private final float xOffsetMm;
    private final float yOffsetMm;
    private final float yawScalar;

    public OdometrySubsystem(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, DEVICE_NAME);

        pinpoint.setOffsets(X_POD_OFFSET_MM, Y_POD_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(POD_TYPE);
        pinpoint.setEncoderDirections(X_ENCODER_DIRECTION, Y_ENCODER_DIRECTION);

        // Robot must be stationary during init for this to calibrate correctly.
        pinpoint.resetPosAndIMU();

        deviceId = pinpoint.getDeviceID();
        deviceVersion = pinpoint.getDeviceVersion();
        xOffsetMm = pinpoint.getXOffset(DistanceUnit.MM);
        yOffsetMm = pinpoint.getYOffset(DistanceUnit.MM);
        yawScalar = pinpoint.getYawScalar();
    }

    /** Call once per loop before reading any position/heading/velocity data. */
    public void update() {
        pinpoint.update();
    }

    /** Zeroes position and recalibrates the IMU. Robot must be stationary when called. */
    public void resetPosAndIMU() {
        pinpoint.resetPosAndIMU();
    }

    /** Recalibrates the IMU only, keeping the current tracked position. Robot must be stationary when called. */
    public void recalibrateIMU() {
        pinpoint.recalibrateIMU();
    }

    public Pose2D getPosition() {
        return pinpoint.getPosition();
    }

    public double getHeadingDegrees() {
        return pinpoint.getHeading(AngleUnit.DEGREES);
    }

    public double getVelXMmPerSec() {
        return pinpoint.getVelX(DistanceUnit.MM);
    }

    public double getVelYMmPerSec() {
        return pinpoint.getVelY(DistanceUnit.MM);
    }

    public GoBildaPinpointDriver.DeviceStatus getStatus() {
        return pinpoint.getDeviceStatus();
    }

    public double getHeadingVelocityDegPerSec() {
        return pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
    }

    public int getEncoderXTicks() {
        return pinpoint.getEncoderX();
    }

    public int getEncoderYTicks() {
        return pinpoint.getEncoderY();
    }

    /** Loop time of the Odometry Computer's own sensor fusion loop, in microseconds. */
    public int getLoopTimeMicros() {
        return pinpoint.getLoopTime();
    }

    /** Loop frequency of the Odometry Computer's own sensor fusion loop, in Hz. */
    public double getFrequencyHz() {
        return pinpoint.getFrequency();
    }

    // Cached at init (see field comments) rather than re-read every loop.

    public int getDeviceId() {
        return deviceId;
    }

    public int getDeviceVersion() {
        return deviceVersion;
    }

    public float getXOffsetMm() {
        return xOffsetMm;
    }

    public float getYOffsetMm() {
        return yOffsetMm;
    }

    public float getYawScalar() {
        return yawScalar;
    }
}
