package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.commands.SetFlywheelRpmCommand;
import org.firstinspires.ftc.teamcode.commands.SetIndexerStateCommand;
import org.firstinspires.ftc.teamcode.commands.SetIntakeStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp(name = "Mecanum TeleOp 67", group = "Drive")
public class MecanumTeleOp extends OpMode {
    // Trigger past this counts as "held".
    private static final double TRIGGER_THRESHOLD = 0.5;

    private final Hardware hardware = new Hardware();
    private final CommandScheduler scheduler = CommandScheduler.getInstance();

    private Drivebase drivebase;
    private Intake intake;
    private Indexer indexer;
    private Flywheel flywheel;
    private Hood hood;
    private Limelight limelight;
    private Turret turret;

    private boolean fieldCentric = true;
    // Tracks the fire trigger so we can stop feeding exactly when it's released.
    private boolean firePrev = false;

    @Override
    public void init(){
        hardware.init(hardwareMap);
        drivebase = new Drivebase(hardware);
        intake = new Intake(hardware);
        indexer = new Indexer(hardware);
        flywheel = new Flywheel(hardware);
        hood = new Hood(hardware);
        limelight = new Limelight(hardware);
        turret = new Turret(hardware, limelight);

        scheduler.reset();
        // Order matters: the Limelight must refresh BEFORE the Turret reads it,
        // so the turret aims on this loop's fresh vision data.
        scheduler.registerSubsystem(intake, indexer, flywheel, hood, limelight, turret);

        telemetry.addLine("Initialized.");
        telemetry.addData("Aiming at", targetName());
        telemetry.update();
    }

    @Override
    public void loop(){
        // Driving (gamepad1).
        if (gamepad1.options) {
            drivebase.resetHeading();
        }
        drivebase.driveWithGamepad(gamepad1, fieldCentric);

        // Alliance goal selection (gamepad1). Pick which AprilTag the turret
        // hunts for — do this once at the start of the match for your alliance.
        if (gamepad1.dpadLeftWasPressed()) {
            limelight.setTargetTagId(Constants.Vision.BLUE_GOAL_TAG);
        }
        if (gamepad1.dpadRightWasPressed()) {
            limelight.setTargetTagId(Constants.Vision.RED_GOAL_TAG);
        }

        // Intake (gamepad2). rising-edge presses schedule state changes.
        if (gamepad2.rightBumperWasPressed()) {
            scheduler.schedule(new SetIntakeStateCommand(intake, Intake.State.INTAKING));
        }
        if (gamepad2.leftBumperWasPressed()) {
            scheduler.schedule(new SetIntakeStateCommand(intake, Intake.State.OUTTAKING));
        }
        if (gamepad2.aWasPressed()) {
            scheduler.schedule(new SetIntakeStateCommand(intake, Intake.State.IDLE));
        }

        // Indexer manual control (gamepad2). Auto-fire below can override this
        // while the fire trigger is held.
        if (gamepad2.dpadUpWasPressed()) {
            scheduler.schedule(new SetIndexerStateCommand(indexer, Indexer.State.FEEDING));
        }
        if (gamepad2.dpadDownWasPressed()) {
            scheduler.schedule(new SetIndexerStateCommand(indexer, Indexer.State.REVERSING));
        }
        if (gamepad2.bWasPressed()) {
            scheduler.schedule(new SetIndexerStateCommand(indexer, Indexer.State.IDLE));
        }

        // Flywheel manual presets (gamepad2). X spins up, Y stops. Auto-aim
        // (below) overrides the target RPM while it's active.
        if (gamepad2.xWasPressed()) {
            scheduler.schedule(new SetFlywheelRpmCommand(flywheel, Constants.Flywheel.SHOOT_RPM));
        }
        if (gamepad2.yWasPressed()) {
            scheduler.schedule(new SetFlywheelRpmCommand(flywheel, 0));
        }

        // --- Turret + shooter integration (gamepad2) ---
        // Hold right trigger: turret auto-aims at the goal AND the flywheel
        // auto-ranges to the measured distance. These run in parallel via the
        // Limelight and Turret subsystem periodics.
        boolean autoAim = gamepad2.right_trigger > TRIGGER_THRESHOLD;
        if (autoAim) {
            turret.setState(Turret.State.AUTO_AIM);

            double distance = limelight.getDistanceMeters();
            if (distance > 0) {
                // Range read is good: auto-range both the flywheel speed and the
                // hood angle to the measured distance.
                flywheel.setTargetRpm(Flywheel.rpmForDistance(distance));
                hood.setForDistance(distance);
            } else {
                // No range read yet — fall back to the flywheel preset and leave
                // the hood wherever the operator last put it.
                flywheel.setTargetRpm(Constants.Flywheel.SHOOT_RPM);
            }
        } else {
            // Not aiming: let the operator nudge the turret with the right stick.
            double manual = gamepad2.right_stick_x;
            if (Math.abs(manual) > Constants.Drive.DEADZONE) {
                turret.setState(Turret.State.MANUAL);
                turret.setManualPower(manual);
            } else {
                turret.setState(Turret.State.IDLE);
            }
        }

        // Manual hood presets (gamepad2): snap to a close/flat or far/steep
        // angle. Useful when shooting without a range read, or to override the
        // auto-ranged angle. dpad up/down are taken by the indexer, so the hood
        // uses dpad left/right.
        if (gamepad2.dpadLeftWasPressed()) {
            hood.setNearPreset();
        }
        if (gamepad2.dpadRightWasPressed()) {
            hood.setFarPreset();
        }

        // Hold left trigger to FIRE — but the indexer only feeds when we're
        // actually locked on AND up to speed, so we never launch a shot that
        // would miss. Releasing the trigger stops the feed.
        boolean fire = gamepad2.left_trigger > TRIGGER_THRESHOLD;
        boolean ready = turret.isOnTarget() && flywheel.atTargetRpm();
        if (fire) {
            indexer.setState(ready ? Indexer.State.FEEDING : Indexer.State.IDLE);
        } else if (firePrev) {
            indexer.setState(Indexer.State.IDLE);
        }
        firePrev = fire;

        scheduler.run();

        telemetry.addData("Mode", fieldCentric ? "Field-centric" : "Robot-centric");
        telemetry.addData("Heading (deg)", Math.toDegrees(drivebase.getHeading()));
        telemetry.addData("Intake", intake.getState());
        telemetry.addData("Indexer", indexer.getState());
        telemetry.addData("Flywheel target", flywheel.getTargetRpm());
        telemetry.addData("Flywheel actual", "%.0f rpm", flywheel.getCurrentRpm());
        telemetry.addData("Flywheel at speed", flywheel.atTargetRpm());
        telemetry.addData("Hood position", "%.2f", hood.getCommandedPosition());
        telemetry.addLine();
        telemetry.addData("Aiming at", targetName());
        telemetry.addData("Turret", turret.getState());
        telemetry.addData("Target visible", limelight.hasTarget());
        if (limelight.hasTarget()) {
            telemetry.addData("tx (deg)", "%.2f", limelight.getTx());
            telemetry.addData("Distance (m)", "%.2f", limelight.getDistanceMeters());
        }
        telemetry.addData("Turret on target", turret.isOnTarget());
        telemetry.addData("READY TO SHOOT", ready);
        telemetry.update();
    }

    private String targetName() {
        return limelight.getTargetTagId() == Constants.Vision.BLUE_GOAL_TAG
                ? "BLUE goal (20)" : "RED goal (24)";
    }

    @Override
    public void stop(){
        scheduler.cancelAll();
        drivebase.stop();
        flywheel.stop();
        hood.stop();
        turret.stop();
        hardware.limelight.stop();
    }
}
