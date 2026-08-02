package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.tuning.VisionTuning;

/**
 * Test 5a — does the camera see the goal tag, and is the distance it reports
 * actually true?
 *
 * <h2>Seeing the tag</h2>
 * The turret only tracks the ONE tag id for your alliance (20 blue / 24 red), so
 * "the Limelight sees tags" is not the same as "the robot has a target". This
 * test lists every id in frame, which separates the three failures cleanly:
 * <ul>
 *   <li><b>No ids at all</b> — pipeline is wrong, exposure is wrong, or the
 *       camera is not actually streaming. Open the Limelight web UI.</li>
 *   <li><b>Ids listed but not yours</b> — you are pointed at an obelisk motif
 *       tag (21/22/23) or the other alliance's goal. Not a code problem.</li>
 *   <li><b>Your id flickers in and out</b> — too far, too steep, or motion blur.
 *       Note the range where it goes unreliable; that is the real limit of
 *       auto-aim, and it is better to learn it now than mid-match.</li>
 * </ul>
 *
 * <h2>Calibrating the distance — do not skip this</h2>
 * Distance is not measured. It is computed from the tag's vertical angle and
 * three numbers that are currently guesses: camera height, tag height, and
 * camera tilt. Those three numbers feed BOTH the auto-ranged flywheel RPM and
 * the auto-ranged hood angle.
 *
 * <p>Which means: if they are wrong, every ranged shot misses by a consistent
 * amount, and it will look exactly like a badly tuned shooting table. You can
 * lose a whole afternoon tuning RPM to compensate for a camera angle that is out
 * by five degrees. Fix the geometry first, then tune the table.
 *
 * <h2>Where to measure from, and to</h2>
 * All three numbers come off the FIELD TILE SURFACE, not the floor under the
 * tiles:
 * <ul>
 *   <li><b>Camera height</b> — tiles to the CENTER OF THE LENS. Not the top of
 *       the case, not the bracket.</li>
 *   <li><b>Tag height</b> — tiles to the CENTER of the AprilTag. Already set to
 *       the DECODE spec (29.5 in / 0.749 m); only change it for a home-built
 *       practice goal.</li>
 *   <li><b>Tape distance</b> — along the floor, from directly under the lens to
 *       directly under the tag center. The formula returns the HORIZONTAL run,
 *       so a tape pulled diagonally up to the tag reads long and sends you
 *       correcting a tilt that was never wrong.</li>
 * </ul>
 *
 * <h2>Solve the tilt, don't hunt for it</h2>
 * The mount angle is the one number you can't sensibly measure with a tool, and
 * it's also the only unknown left once the two heights are known. So don't nudge
 * the bumpers until the number looks right — park at a tape-measured distance,
 * type that distance in with dpad left/right, and press <b>A</b>. It inverts the
 * distance formula for the angle that makes the reported figure land exactly on
 * your tape.
 *
 * <p>That fits ONE point perfectly by construction, which is why it proves
 * nothing on its own. Re-check at a much longer range: tilt errors barely show
 * up close and blow up far away. If the solve was right, the far reading tracks
 * too. If it's still off out there, a HEIGHT is wrong, not the tilt — no single
 * angle can fit two points at different ranges.
 *
 * <p><b>Controls:</b> <b>X</b> target the BLUE goal · <b>B</b> target the RED
 * goal · <b>dpad left/right</b> tape distance +/- 5 cm · <b>A</b> apply the
 * solved tilt · bumpers nudge the tilt +/- 0.5 deg by hand · dpad up/down nudge
 * the camera height +/- 1 cm.
 */
public class VisionTest extends OpMode {

    private final Hardware hardware = new Hardware();
    private final TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();

    private Limelight limelight;

    // What the tape measure says, on the floor, from directly under the lens to
    // directly under the tag center. Typed in with dpad left/right.
    private double tapeM = 1.50;

    @Override
    public void init() {
        hardware.initLimelight(hardwareMap);
        limelight = new Limelight(hardware);
        telemetry.addLine("Limelight test. X = blue goal (20), B = red goal (24).");
        telemetry.addLine("Type the tape distance in with dpad L/R, then press A");
        telemetry.addLine("to solve the mount angle instead of guessing at it.");
        telemetry.update();
    }

    /**
     * The mount angle that would make the reported distance land exactly on the
     * tape measurement, given the two heights.
     *
     * <p>Rearranged straight from the distance formula. If
     * {@code d = (h2 - h1) / tan(a1 + ty)} then the angle that produces a chosen
     * d is {@code a1 = atan((h2 - h1) / d) - ty}. One target sighting and one
     * tape measurement is all it takes; nudging the bumpers until the number
     * looks right is solving the same equation by hand, slowly.
     *
     * <p>Returns NaN when it can't be solved — no tag in frame (no ty), no tape
     * distance entered, or a camera mounted at or above the tag, which makes the
     * geometry degenerate.
     */
    private double solvedMountAngleDeg() {
        double heightDelta = VisionTuning.GOAL_TAG_HEIGHT_M - VisionTuning.CAMERA_HEIGHT_M;
        if (!limelight.hasTarget() || tapeM <= 0 || heightDelta <= 0) {
            return Double.NaN;
        }
        return Math.toDegrees(Math.atan(heightDelta / tapeM)) - limelight.getTy();
    }

    @Override
    public void loop() {
        if (gamepad1.xWasPressed()) {
            limelight.setTargetTagId(Constants.Vision.BLUE_GOAL_TAG);
        }
        if (gamepad1.bWasPressed()) {
            limelight.setTargetTagId(Constants.Vision.RED_GOAL_TAG);
        }
        if (gamepad1.rightBumperWasPressed()) {
            VisionTuning.CAMERA_MOUNT_ANGLE_DEG += 0.5;
        }
        if (gamepad1.leftBumperWasPressed()) {
            VisionTuning.CAMERA_MOUNT_ANGLE_DEG -= 0.5;
        }
        if (gamepad1.dpadUpWasPressed()) {
            VisionTuning.CAMERA_HEIGHT_M += 0.01;
        }
        if (gamepad1.dpadDownWasPressed()) {
            VisionTuning.CAMERA_HEIGHT_M -= 0.01;
        }
        if (gamepad1.dpadRightWasPressed()) {
            tapeM += 0.01;
        }
        if (gamepad1.dpadLeftWasPressed()) {
            tapeM = Math.max(0, tapeM - 0.01);
        }

        limelight.periodic();

        // Solved AFTER periodic() so it uses this loop's ty, not last loop's.
        double solved = solvedMountAngleDeg();
        if (gamepad1.aWasPressed() && !Double.isNaN(solved)) {
            VisionTuning.CAMERA_MOUNT_ANGLE_DEG = solved;
        }

        double distance = limelight.getDistanceMeters();

        telemetry.addData(">> Hunting tag", "%d  (%s)", limelight.getTargetTagId(),
                limelight.getTargetTagId() == Constants.Vision.BLUE_GOAL_TAG ? "BLUE" : "RED");
        telemetry.addData(">> Locked on", limelight.hasTarget());
        telemetry.addData(">> Tags in frame", limelight.getVisibleTagIds().isEmpty()
                ? "none" : limelight.getVisibleTagIds().toString());

        if (limelight.hasTarget()) {
            telemetry.addLine();
            telemetry.addData("tx (left/right)", "%+.2f deg   <- the turret drives this to 0",
                    limelight.getTx());
            telemetry.addData("ty (up/down)", "%+.2f deg   <- distance comes from this",
                    limelight.getTy());
            telemetry.addData("Distance", "%.2f m  =  %.1f in", distance, distance * 39.37);
            telemetry.addData("vs tape", "%+.2f m off", distance - tapeM);
        } else {
            telemetry.addLine();
            telemetry.addData("Last valid frame", "%.0f ms ago", limelight.getStalenessMs());
            telemetry.addLine("No lock. Nothing downstream of here can work:");
            telemetry.addLine("the turret holds still and the shot never arms.");
        }

        telemetry.addLine();
        telemetry.addData("Tape distance", "%.2f m   (dpad left/right)", tapeM);
        telemetry.addData(">> Solve tilt", Double.isNaN(solved)
                ? "-- need a tag in frame and a tape distance"
                : String.format("%.2f deg   (A to apply)", solved));
        telemetry.addLine();
        telemetry.addData("Camera tilt", "%.2f deg   (bumpers to nudge)",
                VisionTuning.CAMERA_MOUNT_ANGLE_DEG);
        telemetry.addData("Camera height", "%.3f m   (dpad up/down, to LENS CENTER)",
                VisionTuning.CAMERA_HEIGHT_M);
        telemetry.addData("Tag height", "%.3f m   (0.749 = DECODE spec, 29.5 in)",
                VisionTuning.GOAL_TAG_HEIGHT_M);
        telemetry.addLine("Measure the tape FLAT ON THE FLOOR, lens to tag —");
        telemetry.addLine("the formula returns horizontal, not the diagonal.");
        telemetry.addLine("Applying fits this ONE point exactly, so always");
        telemetry.addLine("re-check at a much longer range. Still off there?");
        telemetry.addLine("A height is wrong, not the tilt.");
        telemetry.addLine("Copy the final tilt into Constants.Vision.");

        panels.addData("hasTarget", limelight.hasTarget());
        panels.addData("tx", limelight.hasTarget() ? limelight.getTx() : 0.0);
        panels.addData("ty", limelight.hasTarget() ? limelight.getTy() : 0.0);
        panels.addData("distanceM", distance);
        panels.addData("tapeM", tapeM);
        panels.addData("solvedMountAngleDeg", Double.isNaN(solved) ? 0.0 : solved);
        panels.addData("CAMERA_MOUNT_ANGLE_DEG", VisionTuning.CAMERA_MOUNT_ANGLE_DEG);
        panels.addData("CAMERA_HEIGHT_M", VisionTuning.CAMERA_HEIGHT_M);
        panels.addData("GOAL_TAG_HEIGHT_M", VisionTuning.GOAL_TAG_HEIGHT_M);
        panels.update(telemetry);
    }

    @Override
    public void stop() {
        hardware.limelight.stop();
    }
}
