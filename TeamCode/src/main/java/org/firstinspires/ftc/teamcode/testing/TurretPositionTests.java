package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.hardware.Hardware;

/**
 * Simple fixed-position turret checks used before vision testing. Each test
 * initializes without writing PWM; it sends its one position only after START.
 * The Axon's programmed Servo Mode limits remain the physical cable protection.
 */
abstract class TurretPositionTest extends OpMode {
    private final Hardware hardware = new Hardware();

    protected abstract double position();
    protected abstract String positionName();

    @Override
    public void init() {
        hardware.initTurret(hardwareMap);
        telemetry.addData("Turret position test", positionName());
        addPwmTelemetry();
        telemetry.addLine("No movement during INIT. START commands the position.");
        telemetry.addLine("Keep a hand on STOP and watch the wiring.");
        telemetry.update();
    }

    @Override
    public void start() {
        commandPosition();
    }

    @Override
    public void loop() {
        commandPosition();
        telemetry.addData(">> Command", "%s  (input %.2f)", positionName(), position());
        addPwmTelemetry();
        telemetry.addData("Neutral input", "%.2f", Constants.Turret.NEUTRAL_POSITION);
        telemetry.addLine("STOP returns to neutral.");
        telemetry.update();
    }

    @Override
    public void stop() {
        // Do not leave the turret holding against an end limit after a test.
        hardware.turret.setPosition(Constants.Turret.NEUTRAL_POSITION);
    }

    private void commandPosition() {
        hardware.turret.setPosition(position());
    }

    /**
     * Confirms the PWM range the Control Hub reports after Hardware.initTurret()
     * has configured it. This is a readback of the Hub configuration, not an
     * estimate of the servo's physical position.
     */
    private void addPwmTelemetry() {
        if (!(hardware.turret instanceof PwmControl)) {
            telemetry.addLine("!! Servo port does not expose PWM-range readback");
            return;
        }

        PwmControl.PwmRange range = ((PwmControl) hardware.turret).getPwmRange();
        double commandedPulseUs = range.usPulseLower
                + position() * (range.usPulseUpper - range.usPulseLower);
        telemetry.addData("Hub PWM", "%.0f..%.0f us, frame %.1f ms",
                range.usPulseLower, range.usPulseUpper, range.usFrame / 1000.0);
        telemetry.addData("Commanded pulse", "%.0f us", commandedPulseUs);
    }
}

/** Commands the Axon-programmed neutral pose. */
final class TurretNeutralTest extends TurretPositionTest {
    @Override
    protected double position() {
        return Constants.Turret.NEUTRAL_POSITION;
    }

    @Override
    protected String positionName() {
        return "NEUTRAL";
    }
}

/** Commands the low FTC Servo input; the Axon must stop at its programmed left limit. */
final class TurretMaxLeftTest extends TurretPositionTest {
    @Override
    protected double position() {
        return Constants.Turret.MIN_POSITION;
    }

    @Override
    protected String positionName() {
        return "MAX LEFT";
    }
}

/** Commands the high FTC Servo input; the Axon must stop at its programmed right limit. */
final class TurretMaxRightTest extends TurretPositionTest {
    @Override
    protected double position() {
        return Constants.Turret.MAX_POSITION;
    }

    @Override
    protected String positionName() {
        return "MAX RIGHT";
    }
}
