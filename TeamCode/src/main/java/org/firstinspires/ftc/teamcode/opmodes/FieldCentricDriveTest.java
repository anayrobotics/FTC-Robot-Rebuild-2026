package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;

/**
 * Standalone bring-up test for the DRIVEBASE ONLY — the four mecanum drive
 * motors and the navX IMU. Uses {@link Hardware#initDriveOnly} so it runs on a
 * Control Hub configured with just those devices (no intake / indexer /
 * flywheel / turret / Limelight, and NOT the built-in hub IMU). Heading comes
 * entirely from the navX.
 *
 * <p>This is the OpMode to run when bringing up the drivebase on its own.
 * Everything is on gamepad1:
 * <ul>
 *   <li><b>Left stick</b> — translate (field-relative when field-centric is on).</li>
 *   <li><b>Right stick X</b> — rotate.</li>
 *   <li><b>Options</b> — re-zero the field-forward heading. Point the robot
 *       downfield (the direction "away" should mean) and press this at the
 *       start so field-centric forward matches the field.</li>
 *   <li><b>A</b> — toggle field-centric / robot-centric.</li>
 * </ul>
 *
 * <p>FIELD-CENTRIC CHECK: with field-centric on, hold the left stick forward
 * and rotate the robot in place with the right stick — the robot should keep
 * driving the same direction across the floor. If pushing forward drives the
 * wrong way as you rotate, flip {@link
 * org.firstinspires.ftc.teamcode.localization.NavXIMU#INVERT}.
 */
@TeleOp(name = "Field-Centric Drive Test", group = "Test")
public class FieldCentricDriveTest extends OpMode {

    private final Hardware hardware = new Hardware();
    private Drivebase drivebase;

    private boolean fieldCentric = true;

    @Override
    public void init() {
        // Minimal init — only the four drive motors and the navX IMU.
        hardware.initDriveOnly(hardwareMap);
        drivebase = new Drivebase(hardware);

        telemetry.addLine("Drivebase test ready (navX heading).");
        telemetry.addLine("Left stick = drive, right stick X = turn.");
        telemetry.addLine("Options = re-zero heading, A = toggle field/robot centric.");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) {
            fieldCentric = !fieldCentric;
        }
        if (gamepad1.options) {
            drivebase.resetHeading();
        }

        drivebase.driveWithGamepad(gamepad1, fieldCentric);

        telemetry.addData("Mode", fieldCentric ? "Field-centric" : "Robot-centric");
        telemetry.addData("navX calibrating", hardware.navxImu.isCalibrating());
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(drivebase.getHeading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        drivebase.stop();
    }
}
