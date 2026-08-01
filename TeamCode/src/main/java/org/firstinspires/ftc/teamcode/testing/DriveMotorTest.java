package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.hardware.Hardware;

/**
 * Test 1a — spin ONE drive motor at a time and watch its encoder.
 *
 * <p>Four questions, and this is the only test that can answer them cleanly,
 * because with all four running you cannot tell which one is misbehaving:
 * <ol>
 *   <li><b>Is each motor plugged into the port its config name claims?</b>
 *       Select "Front Left", pull the trigger, and the front-left wheel should
 *       be the one that moves. Two swapped cables produce a robot that drives
 *       diagonally when asked to go forward, which is easy to misread as a
 *       mecanum roller-direction problem.</li>
 *   <li><b>Does each wheel spin FORWARD for positive power?</b> Looking at the
 *       robot from above, every wheel's top surface should travel toward the
 *       front. If one is backwards, flip that side in
 *       {@code Constants.Drive.LEFT_DIRECTION} / {@code RIGHT_DIRECTION}.</li>
 *   <li><b>Is every encoder cable plugged in?</b> The ticks for the selected
 *       motor must change while it spins. A motor whose ticks stay at 0 has no
 *       encoder — and in RUN_USING_ENCODER the hub responds to "zero velocity"
 *       by commanding full power, so the robot lurches. This is the single most
 *       common cause of a mecanum that veers hard to one side.</li>
 *   <li><b>Does any motor draw far more current than the others?</b> A wheel
 *       fighting a bent bracket or a rubbing chain shows up here, at maybe 3-4x
 *       its neighbours, long before it cooks the motor.</li>
 * </ol>
 *
 * <p><b>Controls:</b> bumpers pick the motor · right trigger forward · left
 * trigger reverse · dpad up/down power step · <b>A</b> toggles the run mode ·
 * <b>Y</b> zeroes the encoders.
 *
 * <p><b>Put the robot on blocks.</b> One wheel at full power on the floor will
 * spin the robot around.
 */
public class DriveMotorTest extends OpMode {

    private static final String[] NAMES = {"Front Left", "Front Right", "Back Left", "Back Right"};

    private final Hardware hardware = new Hardware();
    private DcMotorEx[] motors;

    private int selected = 0;
    private double power = 0.3;
    private boolean usingEncoder = true;

    @Override
    public void init() {
        hardware.initDrive(hardwareMap);
        motors = new DcMotorEx[]{
                hardware.frontLeft, hardware.frontRight, hardware.backLeft, hardware.backRight};
        telemetry.addLine("Drive motor check. PUT THE ROBOT ON BLOCKS.");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.rightBumperWasPressed()) {
            selected = (selected + 1) % motors.length;
        }
        if (gamepad1.leftBumperWasPressed()) {
            selected = (selected + motors.length - 1) % motors.length;
        }
        if (gamepad1.dpadUpWasPressed()) {
            power = Math.min(1.0, power + 0.1);
        }
        if (gamepad1.dpadDownWasPressed()) {
            power = Math.max(0.1, power - 0.1);
        }
        if (gamepad1.aWasPressed()) {
            usingEncoder = !usingEncoder;
            setRunMode(usingEncoder
                    ? DcMotor.RunMode.RUN_USING_ENCODER
                    : DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (gamepad1.yWasPressed()) {
            setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            setRunMode(usingEncoder
                    ? DcMotor.RunMode.RUN_USING_ENCODER
                    : DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Only the selected motor ever gets power, so whatever moves is the
        // motor you picked — that is the whole point of the test.
        double command = gamepad1.right_trigger * power - gamepad1.left_trigger * power;
        for (int i = 0; i < motors.length; i++) {
            motors[i].setPower(i == selected ? command : 0.0);
        }

        telemetry.addData(">> Motor", "%s   (bumpers to change)", NAMES[selected]);
        telemetry.addData(">> Power", "%.1f  commanded %.2f   (dpad up/down)", power, command);
        telemetry.addData(">> Run mode", "%s   (A to toggle)",
                usingEncoder ? "RUN_USING_ENCODER" : "RUN_WITHOUT_ENCODER");
        telemetry.addLine();
        telemetry.addLine("Wheel should spin FORWARD on right trigger.");
        telemetry.addLine("Ticks for the selected motor MUST change while it spins.");
        telemetry.addLine();

        for (int i = 0; i < motors.length; i++) {
            telemetry.addData(i == selected ? "> " + NAMES[i] : "  " + NAMES[i],
                    "%6d ticks | %7.0f tick/s | %.2f A",
                    motors[i].getCurrentPosition(),
                    motors[i].getVelocity(),
                    motors[i].getCurrent(CurrentUnit.AMPS));
        }

        telemetry.addLine();
        telemetry.addData("Match run mode", Constants.Drive.RUN_MODE);
        telemetry.addLine("If a motor has no encoder, set Constants.Drive.RUN_MODE");
        telemetry.addLine("to RUN_WITHOUT_ENCODER or the robot will veer.");
        telemetry.update();
    }

    private void setRunMode(DcMotor.RunMode mode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(mode);
        }
    }

    @Override
    public void stop() {
        for (DcMotorEx motor : motors) {
            motor.setPower(0);
        }
    }
}
