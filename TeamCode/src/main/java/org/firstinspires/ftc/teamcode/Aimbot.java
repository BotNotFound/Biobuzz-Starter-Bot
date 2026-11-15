package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * Turns the robot to face an AprilTag. The camera points the same way as the shooter, so
 * centering the tag in the image aims the shooter.
 *
 * Usage: call {@link #setEnabled(boolean)} from the controller, and while shooting feed
 * {@link #getTurnPower()} into the drivetrain's rotation input.
 */
public class Aimbot {
    // Alliance tag IDs (tag family 36h11)
    public static final int[] BLUE_TAG_IDS = {38, 39, 40, 41, 42, 43, 44, 45};
    public static final int[] RED_TAG_IDS = {30, 31, 32, 33, 34, 35, 36, 37};

    // Physical side length of the tags' black square. Needed for distance/angle (RPM).
    // PLACEHOLDER - measure your tags and set this!
    public static double TAG_SIZE_INCHES = 6.5;

    // Webcam name in the robot configuration
    public static final String WEBCAM_NAME = "Webcam 1";

    private static final int IMAGE_WIDTH = 640;
    private static final int IMAGE_HEIGHT = 480;

    // Logitech C270 intrinsics at 640x480 (from teamwebcamcalibrations.xml)
    private static final double FX = 822.317;
    private static final double FY = 822.317;
    private static final double CX = 319.495;
    private static final double CY = 242.502;
    // Horizontal half field of view in radians: an error of +/-1 is the image edge
    private static final double HALF_FOV = Math.atan2(IMAGE_WIDTH / 2.0, FX);

    // Proportional gain on the normalized horizontal error (-1..1) and output limits
    public static double kP = 0.7;
    public static double minTurn = 0.12;   // overcome mecanum static friction
    public static double maxTurn = 0.6;
    public static double deadband = 0.03;  // normalized error considered "aligned"

    // Shooter velocity model: velocity = base + perInch * range + perDegree * |yaw|, clamped.
    // range = distance to the tag (inches), yaw = angle of the robot off the tag's perpendicular
    // (degrees). These are STARTING GUESSES - tune them on the field.
    public static double baseVelocity = 1000;
    public static double velocityPerInch = 4.0;
    public static double velocityPerDegree = 2.0;
    public static double minVelocity = 1000;
    public static double maxVelocity = 1800;
    // Keep using the last measured distance this long (ms) after the tag is lost
    public static double poseHoldMs = 500;

    private final VisionPortal portal;
    private final AprilTagProcessor tagProcessor;
    private final int[] targetIds;

    private boolean enabled = false;
    private boolean tagVisible = false;
    private boolean aligned = false;
    private double error = 0;
    private double lastRange = Double.NaN;  // inches
    private double lastYaw = 0;             // degrees
    private long lastPoseTimeMs = 0;

    public Aimbot(HardwareMap hardwareMap, int[] targetIds) {
        this.targetIds = targetIds;

        AprilTagLibrary.Builder library = new AprilTagLibrary.Builder();
        for (int id : targetIds) {
            library.addTag(id, "Tag " + id, TAG_SIZE_INCHES, DistanceUnit.INCH);
        }

        tagProcessor = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(library.build())
                .setLensIntrinsics(FX, FY, CX, CY)
                .build();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .setCameraResolution(new Size(IMAGE_WIDTH, IMAGE_HEIGHT))
                .addProcessor(tagProcessor)
                .build();
    }

    public void setEnabled(boolean enabled) {
        if (this.enabled == enabled) return;
        this.enabled = enabled;
        if (!enabled) {
            lastRange = Double.NaN;
            tagVisible = false;
            aligned = false;
            error = 0;
        }
    }

    public boolean isEnabled() {
        return enabled;
    }

    public boolean isTagVisible() {
        return tagVisible;
    }

    public boolean isAligned() {
        return aligned;
    }

    private boolean isTargetId(int id) {
        for (int t : targetIds) {
            if (t == id) return true;
        }
        return false;
    }

    /** Reads the latest detections. Call once per loop while Aimbot is enabled. */
    public void update() {
        tagVisible = false;
        if (!enabled) return;

        // Of the alliance tags in view, aim at the one closest to the image center
        AprilTagSingleDetection target = null;
        for (AprilTagDetection d : tagProcessor.getDetections()) {
            if (!(d instanceof AprilTagSingleDetection) || !isTargetId(((AprilTagSingleDetection) d).id)) {
                continue;
            }
            AprilTagSingleDetection single = (AprilTagSingleDetection) d;
            if (target == null
                    || Math.abs(single.center.x - CX) < Math.abs(target.center.x - CX)) {
                target = single;
            }
        }
        if (target == null) return;
        tagVisible = true;

        if (target.ftcPose != null) {
            lastRange = target.ftcPose.range;
            lastYaw = target.ftcPose.yaw;
            lastPoseTimeMs = System.currentTimeMillis();
        }

        // Horizontal angle to the tag from the optical axis (right = positive), via the camera
        // intrinsics, normalized by the half FOV so the image edge is about +/-1
        error = Math.atan2(target.center.x - CX, FX) / HALF_FOV;
    }

    /**
     * Rotation power to center the target tag (uses the last {@link #update()}). Positive =
     * counterclockwise (Pedro convention). Returns 0 if disabled, the tag isn't seen, or the
     * robot is already aligned.
     */
    public double getTurnPower() {
        aligned = false;
        if (!enabled || !tagVisible) return 0;

        if (Math.abs(error) < deadband) {
            aligned = true;
            return 0;
        }

        // Tag on the right -> turn right (positive on this robot's drivetrain)
        double turn = kP * error;
        double magnitude = Math.min(maxTurn, Math.max(minTurn, Math.abs(turn)));
        return Math.copySign(magnitude, turn);
    }

    /** True if a recent tag distance measurement is available. */
    public boolean hasRange() {
        return enabled && !Double.isNaN(lastRange)
                && System.currentTimeMillis() - lastPoseTimeMs <= poseHoldMs;
    }

    public double getRange() {
        return lastRange;
    }

    public double getYaw() {
        return lastYaw;
    }

    /**
     * Shooter velocity for the current distance and angle to the tag, or {@code fallback} if the
     * tag distance isn't known.
     */
    public double getShooterVelocity(double fallback) {
        if (!hasRange()) return fallback;
        double v = baseVelocity + velocityPerInch * lastRange + velocityPerDegree * Math.abs(lastYaw);
        return Math.min(maxVelocity, Math.max(minVelocity, v));
    }

    public double getError() {
        return error;
    }

    public void close() {
        portal.close();
    }
}
