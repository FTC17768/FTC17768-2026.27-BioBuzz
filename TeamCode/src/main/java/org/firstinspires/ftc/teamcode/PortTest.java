package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Runs one mechanism at a time at low power, to check each configured name is on the right port.
 * Doesn't use the shooter-ready check on the flapper, so keep game pieces out while testing.
 */
@SuppressWarnings("unused") // discovered via reflection by the FTC Driver Station, not referenced directly
@TeleOp(name="Port Test", group="TEST")
public class PortTest extends LinearOpMode {

    private static final double TEST_POWER = 0.3;

    private static final String[] NAMES = {"INTAKE", "SHOOTER", "intake-left", "intake-right", "flapper"};
    private static final String[] BUTTONS = {"A", "B", "X", "Y", "Right bumper"};

    @Override
    public void runOpMode() {
        // Motors and continuous rotation servos can both be run as a DcMotorSimple.
        // tryGet leaves a device null instead of crashing, so a wrong name shows up as MISSING.
        DcMotorSimple[] devices = new DcMotorSimple[NAMES.length];
        for (int i = 0; i < NAMES.length; i++) {
            devices[i] = hardwareMap.tryGet(DcMotorSimple.class, NAMES[i]);
        }

        while (opModeInInit()) {
            showStatus(devices, new boolean[NAMES.length]);
        }

        while (opModeIsActive()) {
            boolean[] held = {gamepad1.a, gamepad1.b, gamepad1.x, gamepad1.y, gamepad1.right_bumper};
            double power = gamepad1.left_bumper ? -TEST_POWER : TEST_POWER;

            for (int i = 0; i < devices.length; i++) {
                if (devices[i] != null) {
                    devices[i].setPower(held[i] ? power : 0);
                }
            }

            showStatus(devices, held);
        }
    }

    private void showStatus(DcMotorSimple[] devices, boolean[] held) {
        telemetry.addLine("Hold one button at a time. Add left bumper to reverse.");
        for (int i = 0; i < NAMES.length; i++) {
            String status = devices[i] == null ? "MISSING" : (held[i] ? "RUNNING" : "ready");
            telemetry.addData(BUTTONS[i] + ": " + NAMES[i], status);
        }
        telemetry.update();
    }
}
