package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.DriveToPose;
import org.firstinspires.ftc.teamcode.tuning.DriveTuning;

/**
 * Drive-to-pose PID tuning OpMode for the mecanum base, wired to Panels.
 *
 * <p>Edit {@code DriveTuning.FORWARD_*}, {@code STRAFE_*}, and {@code HEADING_*}
 * live in the Panels "Configurables" tab while this runs — {@link DriveToPose}
 * reads them every loop. Move {@code TARGET_X/Y/HEADING_DEG} around to test
 * forward, strafe, diagonal, and turning moves, then watch each axis's
 * {@code *Error} drive toward 0 in the Panels graph view. Tune each axis's kP
 * first, then kD to damp the overshoot; leave kI at 0 unless there's a steady
 * offset.
 *
 * <p><b>Pose comes from PedroPathing's localizer</b> ({@link Constants#createFollower}),
 * so this is only as accurate as that localizer — the odometry pods and their
 * tuning values in {@code pedroPathing/Constants} must be configured first, same
 * as for path following. The follower is used for localization only; this
 * OpMode overwrites the motor commands with its own PID output each loop.
 *
 * <p>Controls: <b>A</b> drive to the target pose, <b>B</b> stop.
 */
@TeleOp(name = "Drive To Pose Tuning", group = "Tuning")
public class DriveToPoseTuningOpMode extends OpMode {

    private final Hardware hardware = new Hardware();
    private final TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();

    private Follower follower;
    private Drivebase drivebase;
    private DriveToPose driveToPose;

    private boolean driving = false;

    @Override
    public void init() {
        hardware.init(hardwareMap);
        drivebase = new Drivebase(hardware);
        driveToPose = new DriveToPose(drivebase);

        follower = Constants.createFollower(hardwareMap);
        // Pose estimate origin. The robot must physically start here for the
        // reported X/Y/heading to be meaningful.
        follower.setStartingPose(new Pose(0, 0, 0));

        panels.debug("Drive-to-pose tuning ready. A = go to target, B = stop.");
        panels.update(telemetry);
    }

    @Override
    public void loop() {
        // Update localization first, then read the fresh pose estimate.
        follower.update();
        Pose pose = follower.getPose();

        if (gamepad1.a) {
            driving = true;
        }
        if (gamepad1.b) {
            driving = false;
            driveToPose.stop();
        }

        boolean atTarget = false;
        if (driving) {
            // Runs the drive PIDs with the live gains and commands the base. This
            // sets the motor powers AFTER follower.update(), so it wins over any
            // command the follower issued this loop.
            atTarget = driveToPose.update(
                    pose.getX(), pose.getY(), pose.getHeading(),
                    DriveTuning.TARGET_X, DriveTuning.TARGET_Y,
                    Math.toRadians(DriveTuning.TARGET_HEADING_DEG));
        } else {
            drivebase.stop();
        }

        // Target vs actual per axis are plotted over time in the Panels graph.
        panels.addData("targetX", DriveTuning.TARGET_X);
        panels.addData("x", pose.getX());
        panels.addData("targetY", DriveTuning.TARGET_Y);
        panels.addData("y", pose.getY());
        panels.addData("targetHeading", DriveTuning.TARGET_HEADING_DEG);
        panels.addData("heading", Math.toDegrees(pose.getHeading()));
        panels.addData("forwardError", driveToPose.getForwardError());
        panels.addData("strafeError", driveToPose.getStrafeError());
        panels.addData("headingError", driveToPose.getHeadingErrorDeg());
        panels.addData("distanceError", driveToPose.getPositionError());
        panels.addData("atTarget", atTarget);
        panels.update(telemetry);
    }

    @Override
    public void stop() {
        drivebase.stop();
    }
}
