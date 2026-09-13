package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class VisionSubsystem {
    private final Limelight3A lime;

    public VisionSubsystem(HardwareMap hardwareMap) {
        lime = hardwareMap.get(Limelight3A.class, "lime");
        lime.pipelineSwitch(0); // Adjust to your desired pipeline index
    }

    public void start() {
        lime.start();
    }

    public void stop() {
        lime.stop();
    }

    /**
     * Retrieves the most recent result from the Limelight.
     * @return The latest LLResult, or null if none is available.
     */
    public LLResult getLatestResult() {
        return lime.getLatestResult();
    }
}