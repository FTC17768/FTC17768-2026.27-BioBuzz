package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveSubsystem {
    private final DcMotor rfd;
    private final DcMotor lfd;
    private final DcMotor rbd;
    private final DcMotor lbd;

    public DriveSubsystem(HardwareMap hardwareMap) {
        // Initialize motors
        rfd = hardwareMap.get(DcMotor.class, "RFD");
        rbd = hardwareMap.get(DcMotor.class, "RBD");
        lfd = hardwareMap.get(DcMotor.class, "LFD");
        lbd = hardwareMap.get(DcMotor.class, "LBD");

        // Direction configuration
        rbd.setDirection(DcMotor.Direction.REVERSE);
        rfd.setDirection(DcMotor.Direction.REVERSE);
        
        // Optional: Set zero power behavior
        rfd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lfd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Drives the robot using mecanum kinematics.
     * @param forward Forward/backward input (typically left stick Y)
     * @param strafe Left/right strafing input (typically left stick X)
     * @param steer Rotation input (typically right stick X)
     * @param speedMultiplier Multiplier to scale the overall speed
     */
    public void drive(double forward, double strafe, double steer, double speedMultiplier) {
        double rfdPower = speedMultiplier * (forward + (steer + strafe));
        double lfdPower = speedMultiplier * (forward - (steer - strafe));
        double rbdPower = speedMultiplier * (forward + (steer - strafe));
        double lbdPower = speedMultiplier * (forward - (steer + strafe));

        rfd.setPower(rfdPower);
        lfd.setPower(lfdPower);
        rbd.setPower(rbdPower);
        lbd.setPower(lbdPower);
    }
}
