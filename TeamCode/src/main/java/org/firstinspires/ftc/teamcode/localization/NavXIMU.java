package org.firstinspires.ftc.teamcode.localization;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Heading source backed by the Kauai Labs navX2-Micro, read through the FTC
 * SDK's built-in {@link NavxMicroNavigationSensor} driver (no external library
 * needed — the navX2 is backward-compatible with the classic navX-Micro driver).
 *
 * <p>Heading is CCW-positive radians, matching what field-centric drive expects.
 * This is the ONLY heading source on the drivebase — the Control Hub's built-in
 * IMU is deliberately not used.
 *
 * <p>{@code resetYaw()} uses a software offset (rather than the sensor's own
 * zero) so it works regardless of driver support and never blocks.
 */
public class NavXIMU {

    /**
     * Flip if heading DECREASES when the robot rotates counter-clockwise. The
     * correct sign depends on how the navX is mounted (which face is up).
     * VERIFY on the robot: rotate the robot CCW and confirm the reported
     * heading increases.
     */
    public static boolean INVERT = false;

    // The navX auto-calibrates on power-up; readings are invalid until it
    // finishes. Bound the wait so a mis-wired sensor can't hang init forever.
    private static final double CALIBRATION_TIMEOUT_S = 5.0;

    private NavxMicroNavigationSensor navx;
    private IntegratingGyroscope gyro;
    private double yawOffsetRad = 0.0;

    // Grab the sensor and wait out its power-on calibration (bounded).
    public void initialize(HardwareMap hardwareMap, String hardwareMapName) {
        navx = hardwareMap.get(NavxMicroNavigationSensor.class, hardwareMapName);
        gyro = (IntegratingGyroscope) navx;

        long start = System.nanoTime();
        while (navx.isCalibrating()
                && (System.nanoTime() - start) / 1e9 < CALIBRATION_TIMEOUT_S) {
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }
    }

    // True while the sensor is still calibrating (heading not yet trustworthy).
    public boolean isCalibrating() {
        return navx.isCalibrating();
    }

    // Current heading in radians, CCW-positive, normalized to (-pi, pi],
    // relative to the last resetYaw().
    public double getHeading() {
        return AngleUnit.normalizeRadians(readRawYawRad() - yawOffsetRad);
    }

    // Zero the heading at the robot's current orientation (software offset).
    public void resetYaw() {
        yawOffsetRad = readRawYawRad();
    }

    private double readRawYawRad() {
        // firstAngle of an INTRINSIC ZYX decomposition is yaw (rotation about Z).
        Orientation o = gyro.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        double yaw = o.firstAngle;
        return INVERT ? -yaw : yaw;
    }
}
