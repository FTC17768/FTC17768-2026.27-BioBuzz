package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;

@SuppressWarnings("unused") // discovered by AutoTune through the @Tuner annotation
public class Tuning {

    @Tuner(name = "Mecanum")
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    @Tuner(name = "Pinpoint")
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    @Tuner(name = "Foresight")
    public static Procedure foresightTuner() {
        return new ForesightTuner(
                hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
                hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig));
    }

    // TODO: after the Foresight tuner, replace null with () -> new Foresight(Constants.foresightConfig).
    // Until then only the Driving, Localization, Odometry and Pose tests can run.
    @Tuner(name = "Tests")
    public static Procedure tests() {
        return new Tests(
                hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig),
                hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
                null);
    }
}
