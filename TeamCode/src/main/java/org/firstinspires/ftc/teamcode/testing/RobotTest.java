package org.firstinspires.ftc.teamcode.testing;

import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Every bring-up test on the robot, behind one Driver Station entry.
 *
 * <p>The tests are ordered the way you should actually run them: nothing later
 * in the list can be trusted until everything before it passes. Auto-aim needs a
 * working turret, which needs a working encoder; a shot that misses is a hood
 * problem, a flywheel problem, a distance-estimate problem, or an aim problem,
 * and the only cheap way to tell them apart is to have already proved three of
 * the four. Work top to bottom. See {@code doc/TEST_DAY.md} for what each test
 * is looking for and what to do when it fails.
 *
 * <p><b>Menu controls:</b> dpad up/down to move, right bumper to select, left
 * bumper to go back. Select before pressing play. To get back to the menu once
 * you've picked a test, stop the OpMode and init it again.
 *
 * <p>Each test brings up ONLY the hardware it needs. That is deliberate: a hood
 * servo that isn't plugged in should not stop you testing the drivebase, and
 * ruling a subsystem out of a problem is most of debugging.
 */
@TeleOp(name = "Robot Test", group = "Test")
public class RobotTest extends SelectableOpMode {

    public RobotTest() {
        super("Select a test  (run these in order)", s -> {
            s.add("0 - Hardware Scan", HardwareScanTest::new);

            s.folder("1 - Drivebase", f -> {
                f.add("1a Motor Check (one at a time)", DriveMotorTest::new);
                f.add("1b Drive + Heading", DriveTest::new);
            });

            s.folder("2 - Ball Path", f -> {
                f.add("2a Intake + Indexer", IntakeIndexerTest::new);
                f.add("2b Stopper Gate", StopperTest::new);
            });

            s.folder("3 - Shooter", f -> {
                f.add("3a Flywheel RPM", FlywheelTest::new);
                f.add("3b Hood Angle", HoodTest::new);
            });

            s.folder("4 - Turret", f -> {
                f.add("4a Encoder + Manual + Park (no camera)", TurretManualTest::new);
            });

            s.folder("5 - Vision + Auto-Aim", f -> {
                f.add("5a Limelight + Distance Calibration", VisionTest::new);
                f.add("5b Turret Auto-Aim", TurretAimTest::new);
            });

            s.folder("6 - Whole Robot", f -> {
                f.add("6a Shooter End-to-End (no drive)", ShooterTest::new);
                f.add("6b Drive To Pose", DriveToPoseTest::new);
            });
        });
    }
}
