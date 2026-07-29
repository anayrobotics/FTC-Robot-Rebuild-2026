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

        scheduler.reset();
        scheduler.registerSubsystem(intake, indexer, flywheel, hood);

        telemetry.addLine("Initialized.");
        telemetry.update();
    }

    @Override
    public void loop(){
        // Driving (gamepad1).
        if (gamepad1.options) {
            drivebase.resetHeading();
        }
        drivebase.driveWithGamepad(gamepad1, fieldCentric);

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

        // Flywheel manual presets (gamepad2). X spins up, Y stops.
        if (gamepad2.xWasPressed()) {
            scheduler.schedule(new SetFlywheelRpmCommand(flywheel, Constants.Flywheel.SHOOT_RPM));
        }
        if (gamepad2.yWasPressed()) {
            scheduler.schedule(new SetFlywheelRpmCommand(flywheel, 0));
        }

        // Manual hood presets (gamepad2): snap to a close/flat or far/steep
        // angle. dpad up/down are taken by the indexer, so the hood uses
        // dpad left/right.
        if (gamepad2.dpadLeftWasPressed()) {
            hood.setNearPreset();
        }
        if (gamepad2.dpadRightWasPressed()) {
            hood.setFarPreset();
        }

        // Hold left trigger to FIRE — but the indexer only feeds when the
        // flywheel is up to speed, so we never launch a shot that would miss.
        // Releasing the trigger stops the feed.
        boolean fire = gamepad2.left_trigger > TRIGGER_THRESHOLD;
        boolean ready = flywheel.atTargetRpm();
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
        telemetry.addData("READY TO SHOOT", ready);
        telemetry.update();
    }

    @Override
    public void stop(){
        scheduler.cancelAll();
        drivebase.stop();
        flywheel.stop();
        hood.stop();
    }
}
