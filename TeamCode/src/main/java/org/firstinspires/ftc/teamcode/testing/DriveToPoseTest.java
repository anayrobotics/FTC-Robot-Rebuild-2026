package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveToPose;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.tuning.DriveTuning;

/**
 * Test 6b — drive to a point on the field and tune the three PIDs that get you
 * there.
 *
 * <p><b>Check localization before you touch a gain.</b> This controller is only
 * as good as the pose it is given, and a bad pose looks exactly like bad tuning:
 * the robot creeps, or oscillates, or confidently parks in the wrong place. So
 * first press nothing, push the robot forward a foot by hand, and watch X. It
 * should rise by about 12. Push it left: Y rises. Rotate counter-clockwise:
 * heading rises. If any axis reads backwards or the wrong magnitude, fix the
 * localizer in {@code pedroPathing/Constants} — the ticks-to-inches numbers are
 * untuned guesses.
 *
 * <p>With no odometry pods fitted, pose comes from the drive encoders, so wheel
 * slip goes straight into the estimate and heading drifts. Expect it to wander
 * over a run. Good enough to prove the controller works; not good enough for a
 * real autonomous.
 *
 * <h2>Tuning order</h2>
 * One axis at a time, with the target set so only that axis has to move. Raise
 * kP until it arrives briskly, add kD until it stops overshooting, leave kI at
 * 0. Strafe usually needs more than forward — a mecanum wastes a lot of a
 * sideways push — so tune them separately rather than copying one to the other.
 *
 * <p><b>Controls:</b> hold <b>A</b> to drive to the target (release to stop) ·
 * <b>B</b> stop and reset the PIDs · <b>X</b> re-zero the pose here · dpad
 * left/right nudge TARGET_X · dpad up/down nudge TARGET_Y · bumpers rotate
 * TARGET_HEADING.
 *
 * <p><b>Space.</b> The robot will drive several feet. Clear the floor.
 */
public class DriveToPoseTest extends OpMode {

    private final Hardware hardware = new Hardware();
    private final TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();

    private Follower follower;
    private Drivebase drivebase;
    private DriveToPose driveToPose;

    @Override
    public void init() {
        hardware.initDrive(hardwareMap);
        hardware.initNavx(hardwareMap);
        drivebase = new Drivebase(hardware);
        driveToPose = new DriveToPose(drivebase);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        telemetry.addLine("Drive to pose. Hold A to drive, release to stop.");
        telemetry.addData("Localizer", Constants.localizerName());
        telemetry.update();
    }

    @Override
    public void loop() {
        // Localization first, then read the fresh estimate.
        follower.update();
        Pose pose = follower.getPose();

        if (gamepad1.xWasPressed()) {
            follower.setStartingPose(new Pose(0, 0, 0));
        }
        if (gamepad1.dpadRightWasPressed()) {
            DriveTuning.TARGET_X += 6;
        }
        if (gamepad1.dpadLeftWasPressed()) {
            DriveTuning.TARGET_X -= 6;
        }
        if (gamepad1.dpadUpWasPressed()) {
            DriveTuning.TARGET_Y += 6;
        }
        if (gamepad1.dpadDownWasPressed()) {
            DriveTuning.TARGET_Y -= 6;
        }
        if (gamepad1.rightBumperWasPressed()) {
            DriveTuning.TARGET_HEADING_DEG += 45;
        }
        if (gamepad1.leftBumperWasPressed()) {
            DriveTuning.TARGET_HEADING_DEG -= 45;
        }

        // Held, not latched. A runaway drive-to-pose is stopped by letting go of
        // the stick hand, which is faster than finding the stop button.
        boolean driving = gamepad1.a && !gamepad1.b;
        boolean atTarget = false;
        if (driving) {
            atTarget = driveToPose.update(
                    pose.getX(), pose.getY(), pose.getHeading(),
                    DriveTuning.TARGET_X, DriveTuning.TARGET_Y,
                    Math.toRadians(DriveTuning.TARGET_HEADING_DEG));
        } else {
            driveToPose.stop();
        }

        telemetry.addData(">> ", driving ? (atTarget ? "AT TARGET" : "DRIVING") : "HOLD A TO DRIVE");
        telemetry.addData(">> Target", "x %.0f  y %.0f  heading %.0f deg",
                DriveTuning.TARGET_X, DriveTuning.TARGET_Y, DriveTuning.TARGET_HEADING_DEG);
        telemetry.addData(">> Pose", "x %.1f  y %.1f  heading %.0f deg",
                pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
        telemetry.addLine();
        telemetry.addData("Error", "fwd %+.1f in  strafe %+.1f in  heading %+.1f deg",
                driveToPose.getForwardError(), driveToPose.getStrafeError(),
                driveToPose.getHeadingErrorDeg());
        telemetry.addLine();
        telemetry.addData("Localizer", Constants.localizerName());
        telemetry.addLine("Push the robot by hand FIRST: forward a foot must");
        telemetry.addLine("raise x by ~12, left raises y, CCW raises heading.");

        panels.addData("targetX", DriveTuning.TARGET_X);
        panels.addData("x", pose.getX());
        panels.addData("targetY", DriveTuning.TARGET_Y);
        panels.addData("y", pose.getY());
        panels.addData("targetHeading", DriveTuning.TARGET_HEADING_DEG);
        panels.addData("heading", Math.toDegrees(pose.getHeading()));
        panels.addData("forwardError", driveToPose.getForwardError());
        panels.addData("strafeError", driveToPose.getStrafeError());
        panels.addData("headingError", driveToPose.getHeadingErrorDeg());
        panels.addData("atTarget", atTarget);
        panels.update(telemetry);
    }

    @Override
    public void stop() {
        drivebase.stop();
    }
}
