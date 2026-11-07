package org.firstinspires.ftc.teamcode.pedro;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Loads paths saved by the Pedro Visualizer (visualizer.pedropathing.com), so the saved project file
 * is the only copy of the path numbers. Save projects to TeamCode/src/main/assets/paths/.
 *
 * Supports lines, curves and linear, constant and tangential headings. Anything else stops with an
 * error at init instead of driving a different path from the one that was drawn. Waits added in the
 * visualizer are ignored: robot actions belong in the autonomous code.
 */
public final class PathFile {

    private static final String FOLDER = "paths/";
    private static final String EXTENSION = ".pp.json";
    private static final String SUPPORTED_MAJOR_VERSION = "2.";

    private final String fileName;
    private final Pose startPose;
    private final Map<String, Path> paths;

    private PathFile(String fileName, Pose startPose, Map<String, Path> paths) {
        this.fileName = fileName;
        this.startPose = startPose;
        this.paths = paths;
    }

    /** Loads paths/name.pp.json from the app's assets. */
    public static PathFile load(HardwareMap hardwareMap, String name) {
        String fileName = name + EXTENSION;
        try (InputStream input = hardwareMap.appContext.getAssets().open(FOLDER + fileName)) {
            return parse(fileName, readAll(input));
        } catch (IOException e) {
            throw new IllegalArgumentException("Can't find " + fileName + " in TeamCode/src/main/assets/paths/.", e);
        }
    }

    static PathFile parse(String fileName, String json) {
        try {
            JSONObject project = new JSONObject(json);

            String version = project.optString("version", "");
            if (!version.startsWith(SUPPORTED_MAJOR_VERSION)) {
                throw new IllegalArgumentException(fileName + " is visualizer format version '" + version
                        + "', but PathFile reads version " + SUPPORTED_MAJOR_VERSION + "x.");
            }

            PoseFactory poses = PoseFactory.degrees();
            JSONObject start = project.getJSONObject("startPoint");
            double x = start.getDouble("x");
            double y = start.getDouble("y");
            JSONArray lines = project.getJSONArray("lines");
            Pose startPose = poses.of(x, y, startHeadingDeg(start, lines));

            // Each path starts where the one before it ended.
            Map<String, Path> paths = new LinkedHashMap<>();
            for (int i = 0; i < lines.length(); i++) {
                JSONObject path = lines.getJSONObject(i);
                String name = path.optString("name", "");
                if (name.isEmpty()) {
                    throw new IllegalArgumentException(fileName + ": path " + (i + 1) + " has no name.");
                }
                if (paths.containsKey(name)) {
                    throw new IllegalArgumentException(fileName + ": two paths are named '" + name + "'.");
                }

                paths.put(name, buildPath(fileName, name, path, poses, x, y));

                JSONObject end = path.getJSONObject("endPoint");
                x = end.getDouble("x");
                y = end.getDouble("y");
            }

            return new PathFile(fileName, startPose, paths);
        } catch (JSONException e) {
            throw new IllegalArgumentException(fileName + " isn't a valid visualizer file: " + e.getMessage(), e);
        }
    }

    /**
     * The heading the robot is placed at. The visualizer edits the start heading on the first path and
     * can leave startPoint.headingDeg stale, and a wrong value here makes the robot spin at the start of
     * the path and then drive with the whole field turned around.
     */
    private static double startHeadingDeg(JSONObject start, JSONArray lines) throws JSONException {
        if (lines.length() > 0) {
            JSONObject heading = lines.getJSONObject(0).getJSONObject("heading");
            String type = heading.getString("type");
            if ("linear".equals(type)) {
                return heading.getDouble("startDeg");
            }
            if ("constant".equals(type)) {
                return heading.getDouble("degrees");
            }
        }
        return start.getDouble("headingDeg");
    }

    private static Path buildPath(String fileName, String name, JSONObject path, PoseFactory poses,
                                  double startX, double startY) throws JSONException {
        if (!"atomic".equals(path.optString("kind", "atomic"))) {
            throw unsupported(fileName, name, "grouped paths");
        }
        JSONArray throughPoints = path.optJSONArray("throughPoints");
        if (throughPoints != null && throughPoints.length() > 0) {
            throw unsupported(fileName, name, "through-curves");
        }

        JSONObject end = path.getJSONObject("endPoint");
        double endX = end.getDouble("x");
        double endY = end.getDouble("y");
        JSONObject heading = path.getJSONObject("heading");
        String headingType = heading.getString("type");

        // Pedro takes headings from poses, so linear paths need their headings on the end poses.
        double startDeg = "linear".equals(headingType) ? heading.getDouble("startDeg") : 0;
        double endDeg = "linear".equals(headingType) ? heading.getDouble("endDeg") : 0;
        Pose startPose = poses.of(startX, startY, startDeg);
        Pose endPose = poses.of(endX, endY, endDeg);

        Path built = buildCurve(path.optJSONArray("controlPoints"), poses, startPose, endPose);

        switch (headingType) {
            case "linear":
                return built.linear(startPose, endPose);
            case "constant":
                return built.constant(poses.of(endX, endY, heading.getDouble("degrees")));
            case "tangential":
                return heading.optBoolean("reverse", false) ? built.reverseTangent() : built.tangent();
            default:
                throw unsupported(fileName, name, "'" + headingType + "' headings");
        }
    }

    private static Path buildCurve(JSONArray controlPoints, PoseFactory poses, Pose startPose, Pose endPose)
            throws JSONException {
        if (controlPoints == null || controlPoints.length() == 0) {
            return line(startPose, endPose);
        }

        List<Pose> points = new ArrayList<>();
        points.add(startPose);
        for (int i = 0; i < controlPoints.length(); i++) {
            JSONObject point = controlPoints.getJSONObject(i);
            points.add(poses.of(point.getDouble("x"), point.getDouble("y"), 0));
        }
        points.add(endPose);
        return curve(points.toArray(new Pose[0]));
    }

    private static IllegalArgumentException unsupported(String fileName, String pathName, String feature) {
        return new IllegalArgumentException(fileName + ": path '" + pathName + "' uses " + feature
                + ", which PathFile doesn't support yet.");
    }

    private static String readAll(InputStream input) throws IOException {
        ByteArrayOutputStream bytes = new ByteArrayOutputStream();
        byte[] buffer = new byte[4096];
        int count;
        while ((count = input.read(buffer)) != -1) {
            bytes.write(buffer, 0, count);
        }
        return new String(bytes.toByteArray(), StandardCharsets.UTF_8);
    }

    /** Where the robot must be placed at the start, taken from the file. */
    public Pose startPose() {
        return startPose;
    }

    /** Gets a path by its name in the visualizer. */
    public Path get(String name) {
        Path path = paths.get(name);
        if (path == null) {
            throw new IllegalArgumentException(fileName + " has no path named '" + name
                    + "'. It has: " + paths.keySet());
        }
        return path;
    }
}
