package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

/**
 * Test 0 — is everything in the Robot Configuration, under the right name and
 * the right type?
 *
 * <p>Run this first, every time, before anything else. It is the only test that
 * touches no hardware at all: it looks each device up and catches the failure
 * instead of throwing, so a missing servo produces a line of telemetry rather
 * than a crashed OpMode.
 *
 * <p>This exists because the failure it catches is otherwise very expensive to
 * diagnose. A device missing from the config makes {@code Hardware.init()}
 * throw, the Driver Station shows a stack trace, and every OpMode is dead —
 * including ones that have nothing to do with the missing part. Ten seconds
 * here saves twenty minutes of "why won't anything run".
 *
 * <p>Nothing moves. Safe to run with the robot on a cart.
 */
public class HardwareScanTest extends OpMode {

    private List<Hardware.DeviceCheck> results;

    @Override
    public void init() {
        results = Hardware.scan(hardwareMap);
        report();
    }

    @Override
    public void init_loop() {
        report();
    }

    @Override
    public void loop() {
        report();
    }

    private void report() {
        int missing = 0;
        for (Hardware.DeviceCheck c : results) {
            if (!c.present) {
                missing++;
            }
        }

        if (missing == 0) {
            telemetry.addLine(">> ALL DEVICES PRESENT — go to test 1a");
        } else {
            telemetry.addData(">> MISSING", "%d device%s — fix the Robot Configuration first",
                    missing, missing == 1 ? "" : "s");
        }
        telemetry.addLine();

        String group = null;
        for (Hardware.DeviceCheck c : results) {
            if (!c.group.equals(group)) {
                group = c.group;
                telemetry.addLine("-- " + group);
            }
            telemetry.addData(c.present ? "  OK  " : "  ??  ",
                    "%s (%s)", c.configName, c.type);
        }

        if (missing > 0) {
            telemetry.addLine();
            telemetry.addLine("Missing devices — the name in the Robot Configuration");
            telemetry.addLine("must match EXACTLY, including capitals:");
            for (Hardware.DeviceCheck c : results) {
                if (!c.present) {
                    telemetry.addData("  need", "\"%s\" as %s", c.configName, c.type);
                }
            }
        }

        telemetry.addLine();
        telemetry.addData("Pedro localizer", Constants.localizerName());
        telemetry.update();
    }
}
