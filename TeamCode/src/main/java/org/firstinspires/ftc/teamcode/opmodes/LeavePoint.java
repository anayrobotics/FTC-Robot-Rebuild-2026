package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;

/**
 * The fallback auto: drive off the launch line for the leave points, nothing
 * else.
 *
 * <p>Open loop and about as simple as an OpMode gets — power for a time, then
 * stop for the rest of auto. No vision, no shooting, no pathing, and only the
 * drive motors are touched, so this runs on the robot in whatever state it's in.
 * When the shooter is broken or the field is unfamiliar, this is the one to
 * queue: it can't fail on a missing device or a tag it can't see.
 *
 * <p>For the auto that actually scores, see {@link PreloadAuto}.
 *
 * <h2>Tuning</h2>
 * The two numbers below are the whole routine. {@link #DRIVE_POWER} times
 * {@link #DRIVE_TIME_S} has to clear the line with margin — time it on the real
 * field rather than trusting the default, since carpet and battery both move it.
 */
@Autonomous(name = "Leave Point Auto", group = "Auto")
public class LeavePoint extends OpMode {
    // Forward power and how long to hold it. Positive axial is forwards; flip
    // the sign here if your robot starts pointed the other way.
    private static final double DRIVE_POWER = 0.4;
    private static final double DRIVE_TIME_S = 2.0;

    private final Hardware hardware = new Hardware();

    private Drivebase drivebase;
    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        // Only the drive group. Per-group init rather than hardware.init()
        // because the full init throws on any device missing from the Robot
        // Configuration, and not needing the shooter at all is the point of
        // this auto. The IMU comes along because Drivebase reads it at
        // construction for its field-centric mode, which this OpMode never uses.
        hardware.initDrive(hardwareMap);
        hardware.initImu(hardwareMap);

        drivebase = new Drivebase(hardware);

        telemetry.addLine("Leave Point Auto — drive forward and stop.");
        telemetry.addData("Drive", "%.2f power for %.2f s", DRIVE_POWER, DRIVE_TIME_S);
        telemetry.update();
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        // Robot-centric: heading isn't zeroed to anything meaningful at auto
        // start, so "forward" means forward relative to the robot, which is the
        // only frame we can trust here.
        boolean driving = timer.seconds() < DRIVE_TIME_S;
        if (driving) {
            drivebase.drive(DRIVE_POWER, 0, 0);
        } else {
            drivebase.stop();
        }

        telemetry.addData(">> State", driving ? "DRIVING" : "DONE");
        telemetry.addData("Elapsed", "%.2f / %.2f s", timer.seconds(), DRIVE_TIME_S);
        telemetry.update();
    }

    @Override
    public void stop() {
        drivebase.stop();
    }
}
