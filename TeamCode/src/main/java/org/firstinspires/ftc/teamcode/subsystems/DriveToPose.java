package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.tuning.DriveTuning;
import org.firstinspires.ftc.teamcode.util.PIDFController;

/**
 * Point-to-point ("drive to pose") controller for the mecanum base.
 *
 * <p>Given the robot's current pose and a target pose (both in PedroPathing field
 * coordinates — +X forward, +Y left, heading in radians CCW), it runs three PIDs
 * and commands the {@link Drivebase} to close the gap:
 * <ul>
 *   <li><b>forward</b> and <b>strafe</b> — the field-frame position error rotated
 *       into the robot frame, so one gain always controls forward motion and the
 *       other strafing regardless of which way the robot faces;</li>
 *   <li><b>heading</b> — the shortest-angle error, wrapped to [-pi, pi].</li>
 * </ul>
 *
 * <p>Gains are pulled live from {@link DriveTuning} every {@link #update} so
 * edits made on the Panels dashboard apply immediately (see
 * {@link org.firstinspires.ftc.teamcode.opmodes.DriveToPoseTuningOpMode}). This
 * class owns no pose source of its own — the caller supplies the current pose
 * (the tuning OpMode reads it from PedroPathing's localizer).
 */
public class DriveToPose {
    private final Drivebase drivebase;
    private final PIDFController forwardPid;
    private final PIDFController strafePid;
    private final PIDFController headingPid;

    // Last errors, kept for telemetry / at-target checks (robot frame; inches & rad).
    private double forwardError;
    private double strafeError;
    private double headingError;

    public DriveToPose(Drivebase drivebase) {
        this.drivebase = drivebase;
        // kF is unused for position control (the setpoint is a position, not a
        // velocity), so it stays 0 on all three axes.
        forwardPid = new PIDFController(
                DriveTuning.FORWARD_kP, DriveTuning.FORWARD_kI, DriveTuning.FORWARD_kD, 0);
        strafePid = new PIDFController(
                DriveTuning.STRAFE_kP, DriveTuning.STRAFE_kI, DriveTuning.STRAFE_kD, 0);
        headingPid = new PIDFController(
                DriveTuning.HEADING_kP, DriveTuning.HEADING_kI, DriveTuning.HEADING_kD, 0);
    }

    /**
     * Runs one control step toward {@code target}, commanding the drivebase.
     * All arguments are in field coordinates (inches; heading in radians).
     *
     * @return true once the robot is within the position and heading tolerances.
     */
    public boolean update(double curX, double curY, double curHeading,
                          double targetX, double targetY, double targetHeading) {
        // Refresh gains so live Panels edits take effect this loop.
        forwardPid.setCoefficients(
                DriveTuning.FORWARD_kP, DriveTuning.FORWARD_kI, DriveTuning.FORWARD_kD, 0);
        strafePid.setCoefficients(
                DriveTuning.STRAFE_kP, DriveTuning.STRAFE_kI, DriveTuning.STRAFE_kD, 0);
        headingPid.setCoefficients(
                DriveTuning.HEADING_kP, DriveTuning.HEADING_kI, DriveTuning.HEADING_kD, 0);

        // Field-frame position error, then rotate it into the robot frame so the
        // forward/strafe gains map to the robot's own axes.
        double ex = targetX - curX;
        double ey = targetY - curY;
        double cos = Math.cos(curHeading);
        double sin = Math.sin(curHeading);
        forwardError = ex * cos + ey * sin;   // component along robot forward
        strafeError = ex * sin - ey * cos;    // component along robot right
        headingError = wrapRadians(targetHeading - curHeading);

        // PID with target = error, measured = 0, so the controller's internal
        // error is exactly the error we computed above (kF term is 0).
        double axial = forwardPid.calculate(forwardError, 0);
        double lateral = strafePid.calculate(strafeError, 0);
        double yaw = headingPid.calculate(headingError, 0);

        if (Constants.DriveToPose.INVERT_FORWARD) axial = -axial;
        if (Constants.DriveToPose.INVERT_STRAFE) lateral = -lateral;
        if (Constants.DriveToPose.INVERT_HEADING) yaw = -yaw;

        axial = clamp(axial, Constants.DriveToPose.MAX_DRIVE_POWER);
        lateral = clamp(lateral, Constants.DriveToPose.MAX_DRIVE_POWER);
        yaw = clamp(yaw, Constants.DriveToPose.MAX_TURN_POWER);

        drivebase.drive(axial, lateral, yaw);
        return atTarget();
    }

    public boolean atTarget() {
        double distance = Math.hypot(forwardError, strafeError);
        return distance <= Constants.DriveToPose.POSITION_TOLERANCE_IN
                && Math.abs(headingError) <= Math.toRadians(Constants.DriveToPose.HEADING_TOLERANCE_DEG);
    }

    /** Signed straight-line distance remaining to the target, in inches. */
    public double getPositionError() {
        return Math.hypot(forwardError, strafeError);
    }

    public double getForwardError() {
        return forwardError;
    }

    public double getStrafeError() {
        return strafeError;
    }

    public double getHeadingErrorDeg() {
        return Math.toDegrees(headingError);
    }

    /** Stops the base and clears the PID integrators/derivative history. */
    public void stop() {
        drivebase.stop();
        forwardPid.reset();
        strafePid.reset();
        headingPid.reset();
    }

    private static double wrapRadians(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private static double clamp(double value, double max) {
        return Math.max(-max, Math.min(max, value));
    }
}
