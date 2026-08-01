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
 * <p>Park the robot at a tape-measured distance, compare, and nudge
 * CAMERA_MOUNT_ANGLE_DEG until the reported figure matches. Then check a second,
 * much longer distance — tilt errors barely show up close and blow up far away,
 * so one calibration point proves nothing.
 *
 * <p><b>Controls:</b> <b>X</b> target the BLUE goal · <b>B</b> target the RED
 * goal · bumpers nudge the camera tilt +/- 0.5 deg · dpad up/down nudge the
 * camera height +/- 1 cm.
 */
public class VisionTest extends OpMode {

    private final Hardware hardware = new Hardware();
    private final TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();

    private Limelight limelight;

    @Override
    public void init() {
        hardware.initLimelight(hardwareMap);
        limelight = new Limelight(hardware);
        telemetry.addLine("Limelight test. X = blue goal (20), B = red goal (24).");
        telemetry.update();
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

        limelight.periodic();

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
            telemetry.addLine("Compare against a tape measure. If it disagrees,");
            telemetry.addLine("nudge the tilt with the bumpers until it matches,");
            telemetry.addLine("then re-check at a MUCH longer range.");
        } else {
            telemetry.addLine();
            telemetry.addData("Last valid frame", "%.0f ms ago", limelight.getStalenessMs());
            telemetry.addLine("No lock. Nothing downstream of here can work:");
            telemetry.addLine("the turret holds still and the shot never arms.");
        }

        telemetry.addLine();
        telemetry.addData("Camera tilt", "%.1f deg   (bumpers)", VisionTuning.CAMERA_MOUNT_ANGLE_DEG);
        telemetry.addData("Camera height", "%.3f m   (dpad up/down)", VisionTuning.CAMERA_HEIGHT_M);
        telemetry.addData("Tag height", "%.3f m   (measure it, edit VisionTuning)",
                VisionTuning.GOAL_TAG_HEIGHT_M);
        telemetry.addLine("Copy all three into Constants.Vision when they're right.");

        panels.addData("hasTarget", limelight.hasTarget());
        panels.addData("tx", limelight.hasTarget() ? limelight.getTx() : 0.0);
        panels.addData("ty", limelight.hasTarget() ? limelight.getTy() : 0.0);
        panels.addData("distanceM", distance);
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
