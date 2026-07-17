package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.Subsystem;
import org.firstinspires.ftc.teamcode.hardware.Hardware;

import java.util.List;

// Thin wrapper around the Limelight 3A. It polls the camera once per loop in
// periodic() and caches the horizontal/vertical offset of the ONE AprilTag we
// care about (the alliance goal), so every other subsystem reads a single,
// consistent snapshot instead of hitting the camera itself.
//
// IMPORTANT: register this subsystem with the scheduler BEFORE the Turret so its
// periodic() runs first and the turret aims on this loop's fresh data, not last
// loop's. (CommandScheduler runs periodics in registration order.)
public class Limelight implements Subsystem {
    private final Limelight3A limelight;

    // Which AprilTag id we treat as "the goal". Set per alliance.
    private int targetTagId = Constants.Vision.DEFAULT_TARGET_TAG;

    // Snapshot of the last read, refreshed every periodic().
    private boolean hasTarget = false;
    private double tx = 0;   // horizontal angle to target, degrees (+ = right)
    private double ty = 0;   // vertical angle to target, degrees (+ = up)

    public Limelight(Hardware hardware) {
        limelight = hardware.limelight;
    }

    public void setTargetTagId(int id) {
        targetTagId = id;
    }

    public int getTargetTagId() {
        return targetTagId;
    }

    // True only when the selected goal tag was seen on the most recent frame.
    public boolean hasTarget() {
        return hasTarget;
    }

    // Horizontal error to the goal in degrees. Only meaningful if hasTarget().
    public double getTx() {
        return tx;
    }

    public double getTy() {
        return ty;
    }

    // Estimated straight-line ground distance to the goal, in meters, from the
    // tag's vertical angle. Returns -1 when there is no target or the geometry
    // is degenerate (target at or below the camera's horizontal), so callers
    // must check for a negative result before using it.
    public double getDistanceMeters() {
        if (!hasTarget) {
            return -1;
        }
        double angleRad = Math.toRadians(Constants.Vision.CAMERA_MOUNT_ANGLE_DEG + ty);
        double tan = Math.tan(angleRad);
        if (tan <= 1e-6) {
            return -1;
        }
        double heightDelta = Constants.Vision.GOAL_TAG_HEIGHT_M - Constants.Vision.CAMERA_HEIGHT_M;
        return heightDelta / tan;
    }

    @Override
    public void periodic() {
        LLResult result = limelight.getLatestResult();

        // Assume no target until we actually find our tag in this frame. This is
        // the key safety point: if the camera returns null (not started / USB
        // dropped) or an invalid/empty result, hasTarget stays false and the
        // turret will hold still instead of chasing stale data.
        hasTarget = false;

        if (result == null || !result.isValid()) {
            return;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null) {
            return;
        }

        // Pick out OUR goal tag specifically. result.getTx() would give whatever
        // tag the Limelight considers primary, which may be the wrong goal or an
        // obelisk motif tag, so we match on id instead.
        for (LLResultTypes.FiducialResult fr : fiducials) {
            if (fr.getFiducialId() == targetTagId) {
                tx = fr.getTargetXDegrees();
                ty = fr.getTargetYDegrees();
                hasTarget = true;
                return;
            }
        }
    }
}
