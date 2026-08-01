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
 *
 * <h2>If the navX isn't facing the right way</h2>
 * Three different problems, three different fixes — an offset only solves one
 * of them:
 * <ol>
 *   <li><b>Rotated about the vertical axis</b> (board still lying flat, but
 *       pointing 90°/180°/any angle off from robot-forward). <b>Nothing to
 *       do.</b> The rotation is a constant, and {@code resetYaw()} subtracts
 *       whatever it reads at that moment, so the constant cancels. Just zero
 *       the robot pointing downfield and it's correct.</li>
 *   <li><b>Heading counts backwards</b> (goes DOWN when the robot turns CCW),
 *       which is what you get when the board is mounted upside-down. Set
 *       {@link #INVERT} to {@code true}.</li>
 *   <li><b>Yaw axis isn't vertical</b> — the board is on its edge, mounted to a
 *       side rail or standing up. This one an offset CANNOT fix: yaw is no
 *       longer rotation about the sensor's Z, so {@code firstAngle} below is
 *       reading pitch or roll instead of heading. The symptom is a heading that
 *       barely moves when you spin the robot but swings when you tilt it. Fix
 *       it by remounting the navX flat (much the easiest), or by reading a
 *       different angle out of the {@link Orientation} in
 *       {@code readRawYawRad()}.</li>
 * </ol>
 * Separately, if you need "zero" to mean something other than "where the robot
 * points at init" — an auto that starts facing 90°, say — use
 * {@link #setHeading(double)} instead of {@code resetYaw()}.
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
        setHeading(0.0);
    }

    /**
     * Declares what the robot's heading IS right now, and offsets the sensor so
     * {@link #getHeading()} reports that from here on. {@code resetYaw()} is
     * just {@code setHeading(0)}.
     *
     * <p>Use this when you can't (or don't want to) zero with the robot pointed
     * at field-forward — e.g. an auto that starts on a wall facing 90°: call
     * {@code setHeading(Math.toRadians(90))} at init and field-centric drive and
     * PedroPathing both agree with the real field from the first loop.
     *
     * <p>NOTE this is NOT the knob for a navX that's bolted on rotated. A rigid
     * mounting rotation cancels out of {@code getHeading()} the moment you call
     * {@code resetYaw()} (both the reading and the captured offset shift by the
     * same amount), so a mount rotated about the vertical axis needs no
     * correction at all — just zero it pointing downfield. See {@link #INVERT}
     * for a navX that counts backwards, and the class docs for a navX whose
     * yaw axis isn't vertical.
     *
     * @param headingRad the robot's true heading now, CCW-positive radians.
     */
    public void setHeading(double headingRad) {
        yawOffsetRad = readRawYawRad() - headingRad;
    }

    private double readRawYawRad() {
        // firstAngle of an INTRINSIC ZYX decomposition is yaw (rotation about Z).
        Orientation o = gyro.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        double yaw = o.firstAngle;
        return INVERT ? -yaw : yaw;
    }
}
