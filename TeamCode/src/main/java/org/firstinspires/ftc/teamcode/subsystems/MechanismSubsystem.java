package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MechanismSubsystem {

    // These must match the names in the robot's hardware configuration exactly.
    private static final String INTAKE_MOTOR_NAME = "INTAKE";
    private static final String SHOOTER_MOTOR_NAME = "SHOOTER";
    private static final String INTAKE_LEFT_SERVO_NAME = "intake-left";
    private static final String INTAKE_RIGHT_SERVO_NAME = "intake-right";
    private static final String FLAPPER_SERVO_NAME = "flapper";

    // Low starting powers for first testing. Raise them once each direction is confirmed.
    private static final double INTAKE_POWER = 0.8;
    private static final double SHOOTER_TARGET_VELOCITY = 1320;
    private static final double SHOOTER_VELOCITY_TOLERANCE = 50;
    private static final double FLAPPER_POWER = 0.75;
    private boolean shooterOn = false;

    private final DcMotor intakeMotor;
    private final DcMotorEx shooterMotor;
    private final CRServo intakeLeft;
    private final CRServo intakeRight;
    private final CRServo flapper;

    public MechanismSubsystem(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, INTAKE_MOTOR_NAME);
        shooterMotor = hardwareMap.get(DcMotorEx.class, SHOOTER_MOTOR_NAME);
        intakeLeft = hardwareMap.get(CRServo.class, INTAKE_LEFT_SERVO_NAME);
        intakeRight = hardwareMap.get(CRServo.class, INTAKE_RIGHT_SERVO_NAME);
        flapper = hardwareMap.get(CRServo.class, FLAPPER_SERVO_NAME);

        // Directions confirmed with the Port Test OpMode.
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);
        flapper.setDirection(DcMotorSimple.Direction.REVERSE);

        // Let the shooter spin down on its own instead of braking hard.
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void intakeIn() {
        setIntake(INTAKE_POWER);
    }

    public void intakeOut() {
        setIntake(-INTAKE_POWER);
    }

    public void intakeStop() {
        setIntake(0);
    }

    public void setShooterOn(boolean on) {
        if (on) {
            shooterMotor.setVelocity(SHOOTER_TARGET_VELOCITY);
        } else {
            shooterMotor.setPower(0);
        }
    }

    /** The flywheel speed read from the encoder, in ticks per second. */
    public double getShooterVelocity() {
        return shooterMotor.getVelocity();
    }

    /** True once the shooter has been on for SHOOTER_SPIN_UP_SECONDS. */
    public boolean isShooterReady() {
        return shooterOn && Math.abs(getShooterVelocity() - SHOOTER_TARGET_VELOCITY) <= SHOOTER_VELOCITY_TOLERANCE;
    }

    /** The flapper feeds the shooter, so it only runs when the shooter is ready. */
    public void setFlapperOn(boolean on) {
        flapper.setPower(on && isShooterReady() ? FLAPPER_POWER : 0);
    }

    private void setIntake(double power) {
        intakeMotor.setPower(power);
        intakeLeft.setPower(power);
        intakeRight.setPower(power);
    }
}
