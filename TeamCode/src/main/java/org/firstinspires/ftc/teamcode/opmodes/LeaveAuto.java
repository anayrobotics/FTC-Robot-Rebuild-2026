package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;

/**
 * Minimum-viable autonomous: drive straight off the line, stop, sit still.
 *
 * <p>This exists to bank the LEAVE points and nothing else. It is deliberately
 * the least capable auto in the project, because that is what makes it the most
 * reliable one: it initializes only the four drive motors, so there is no navX,
 * no localizer, no camera, and no servo that can be missing from the robot
 * configuration and take the whole OpMode down at init. If everything else on
 * the robot is broken, this still scores.
 *
 * <p><b>It is open-loop and time-based.</b> Nothing measures where the robot
 * actually ends up — {@link #DRIVE_SECONDS} times {@link #DRIVE_POWER} is the
 * entire model. That is fine for clearing a line by a comfortable margin and is
 * not fine for anything that has to end at a specific spot; use PedroPathing
 * (see {@code PedroAutoExample}) when you need a real position.
 *
 * <p>Distance is more repeatable than open-loop usually deserves, because
 * {@code Constants.Drive.RUN_MODE} is {@code RUN_USING_ENCODER} — the hub holds
 * a velocity rather than a voltage, so a sagging battery slows the robot much
 * less than it would otherwise. It is still worth re-checking the timing on a
 * low battery before you trust it in a match.
 */
@Autonomous(name = "Leave", group = "Auto")
public class LeaveAuto extends OpMode {

    // Slow enough not to wheelie or skid on the start, fast enough to be clear
    // of the line well inside the auto period.
    private static final double DRIVE_POWER = 0.35;

    // TUNE THIS FIRST. Run it on the real field, watch where the robot stops,
    // and adjust until it is comfortably past the line — overshoot costs
    // nothing here, stopping short costs the whole auto.
    private static final double DRIVE_SECONDS = 1.2;

    // Negative drives backwards, for a start position that faces the wrong way.
    // Prefer flipping this over turning the robot round: a straight line is the
    // one move an open-loop auto can be trusted with.
    private static final double DIRECTION = 1.0;

    private final Hardware hardware = new Hardware();
    private Drivebase drivebase;
    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        // Drive only. Anything else initialized here is another device that can
        // be missing from the config and cost us the points this auto exists
        // to guarantee.
        hardware.initDrive(hardwareMap);
        drivebase = new Drivebase(hardware);

        telemetry.addLine("LEAVE auto ready.");
        telemetry.addData("Plan", "drive %s %.1fs at %.0f%% power",
                DIRECTION >= 0 ? "FORWARD" : "BACKWARD", DRIVE_SECONDS, DRIVE_POWER * 100);
        telemetry.addLine("Point the robot where it should end up. It drives straight.");
        telemetry.update();
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        boolean driving = timer.seconds() < DRIVE_SECONDS;
        if (driving) {
            drivebase.drive(DRIVE_POWER * DIRECTION, 0, 0);
        } else {
            // BRAKE zero-power behaviour holds it here for the rest of auto.
            drivebase.stop();
        }

        telemetry.addData(">> State", driving ? "DRIVING" : "DONE — parked");
        telemetry.addData("Elapsed", "%.2f / %.2f s", timer.seconds(), DRIVE_SECONDS);
        // Ticks are here so you can read off how far it actually went and turn
        // this into a distance-based auto later if you want one.
        telemetry.addData("Ticks FL/FR", "%d  %d",
                hardware.frontLeft.getCurrentPosition(), hardware.frontRight.getCurrentPosition());
        telemetry.addData("Ticks BL/BR", "%d  %d",
                hardware.backLeft.getCurrentPosition(), hardware.backRight.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
        drivebase.stop();
    }
}
