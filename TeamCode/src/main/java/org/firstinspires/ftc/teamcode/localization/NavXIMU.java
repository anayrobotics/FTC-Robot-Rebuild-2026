package org.firstinspires.ftc.teamcode.localization;

import com.pedropathing.ftc.localization.CustomIMU;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
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
 * <p>Implements PedroPathing's {@link CustomIMU} so it can drive both the
 * PedroPathing localizer's heading AND our own field-centric drive with one
 * identical convention. Heading is CCW-positive radians, matching what both
 * PedroPathing and {@code Drivebase} expect.
 *
 * <p>{@code resetYaw()} uses a software offset (rather than the sensor's own
 * zero) so it works regardless of driver support and never blocks.
 */
public class NavXIMU implements CustomIMU {

    /**
     * Flip if heading DECREASES when the robot rotates counter-clockwise.
     * The correct sign depends on how the navX is mounted (which face is up).
     * VERIFY on the robot: run PedroPathing's Localization Test, rotate the
     * robot CCW, and confirm the reported heading increases.
     */
    public static boolean INVERT = false;

    // The navX auto-calibrates on power-up; readings are invalid until it
    // finishes. Bound the wait so a mis-wired sensor can't hang init forever.
    private static final double CALIBRATION_TIMEOUT_S = 5.0;

    private NavxMicroNavigationSensor navx;
    private IntegratingGyroscope gyro;
    private double yawOffsetRad = 0.0;

    @Override
    public void initialize(HardwareMap hardwareMap, String hardwareMapName,
                           RevHubOrientationOnRobot hubOrientation) {
        // hubOrientation is ignored: the navX isn't a REV hub IMU, so mounting is
        // handled by INVERT and the sensor's own axes, not RevHubOrientationOnRobot.
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

    @Override
    public double getHeading() {
        return AngleUnit.normalizeRadians(readRawYawRad() - yawOffsetRad);
    }

    @Override
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
