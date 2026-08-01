package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.localization.NavXIMU;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;

/**
 * Test 1b — drive the robot and check the heading sensor.
 *
 * <p>Two things are being proved here, and it matters that they are separate.
 *
 * <p><b>Robot-centric mode</b> proves the mecanum mixing and the motor
 * directions. Push forward: the robot goes forward, not sideways and not in a
 * circle. Push left: it strafes left without rotating. Turn right: it spins in
 * place. If forward works but strafe crabs diagonally, a wheel's rollers are
 * mounted the wrong way round — the four rollers must form an X when you look
 * down at the robot, not four parallel lines.
 *
 * <p><b>Field-centric mode</b> proves the navX, and it can only be trusted once
 * robot-centric is clean. Set the heading zero, drive forward, then rotate the
 * robot 90 degrees by hand and push the stick forward again: the robot must
 * still travel the same way across the field. If it instead drives at a mirror
 * angle, the heading sign is inverted — press X to flip it and try again.
 *
 * <p><b>Controls:</b> left stick translate · right stick X turn · <b>Y</b>
 * field/robot centric · <b>dpad up</b> zero the heading · <b>X</b> flip the
 * heading sign · right bumper held for full speed.
 */
public class DriveTest extends OpMode {

    // Everything is quarter-throttle unless the driver asks otherwise, because
    // the first time you run this you do not yet know which way it will go.
    private static final double SLOW_SCALE = 0.25;

    private final Hardware hardware = new Hardware();
    private Drivebase drivebase;

    private boolean fieldCentric = false;
    

    @Override
    public void init() {
        hardware.initDrive(hardwareMap);
        hardware.initNavx(hardwareMap);
        drivebase = new Drivebase(hardware);
        telemetry.addLine("Drive test ready. Starting in ROBOT-CENTRIC.");
        telemetry.addLine("Prove robot-centric first, then switch with Y.");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.yWasPressed()) {
            fieldCentric = !fieldCentric;
        }
        if (gamepad1.dpadUpWasPressed()) {
            drivebase.resetHeading();

        }
        if (gamepad1.xWasPressed()) {
            // NavXIMU.INVERT is a plain static, so this takes effect immediately
            // — but only for as long as the app is running. The heading offset is
            // measured against the old sign, so re-zero after flipping.
            NavXIMU.INVERT = !NavXIMU.INVERT;
            drivebase.resetHeading();
        }

        double scale = gamepad1.right_bumper ? 1.0 : SLOW_SCALE;
        double axial = -gamepad1.left_stick_y * scale;
        double lateral = gamepad1.left_stick_x * scale;
        double yaw = gamepad1.right_stick_x * scale;

        if (fieldCentric) {
            drivebase.driveFieldCentric(axial, lateral, yaw);
        } else {
            drivebase.drive(axial, lateral, yaw);
        }

        double headingDeg = Math.toDegrees(drivebase.getHeading());

        telemetry.addData(">> Mode", "%s   (Y to switch)",
                fieldCentric ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");
        telemetry.addData(">> Speed", "%.0f%%   (hold right bumper for 100%%)", scale * 100);
        telemetry.addLine();
        telemetry.addData("Heading", "%+.1f deg", headingDeg);
        telemetry.addData("Heading sign", "INVERT = %s   (X to flip)", NavXIMU.INVERT);
        telemetry.addLine("Rotate the robot COUNTER-CLOCKWISE by hand:");
        telemetry.addLine("heading must go UP. If it goes down, press X,");
        telemetry.addLine("then set NavXIMU.INVERT = " + !NavXIMU.INVERT + " in the source.");
        telemetry.addLine();
        telemetry.addData("Stick", "axial %+.2f  lateral %+.2f  yaw %+.2f", axial, lateral, yaw);
        telemetry.addData("FL / FR", "%+.2f  %+.2f",
                hardware.frontLeft.getPower(), hardware.frontRight.getPower());
        telemetry.addData("BL / BR", "%+.2f  %+.2f",
                hardware.backLeft.getPower(), hardware.backRight.getPower());
        telemetry.addLine();
        telemetry.addLine("Robot-centric checks: forward goes forward, strafe");
        telemetry.addLine("does not rotate, turn spins in place.");
        telemetry.update();
    }

    @Override
    public void stop() {
        drivebase.stop();
    }
}
