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
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp(name = "Mecanum TeleOp 67", group = "Drive")
public class MecanumTeleOp extends OpMode {
    private final Hardware hardware = new Hardware();
    private final CommandScheduler scheduler = CommandScheduler.getInstance();

    private Drivebase drivebase;
    private Intake intake;
    private Indexer indexer;
    private Flywheel flywheel;

    private boolean fieldCentric = true;

    @Override
    public void init(){
        hardware.init(hardwareMap);
        drivebase = new Drivebase(hardware);
        intake = new Intake(hardware);
        indexer = new Indexer(hardware);
        flywheel = new Flywheel(hardware);

        scheduler.reset();
        scheduler.registerSubsystem(intake, indexer, flywheel);

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

        // Indexer (gamepad2).
        if (gamepad2.dpadUpWasPressed()) {
            scheduler.schedule(new SetIndexerStateCommand(indexer, Indexer.State.FEEDING));
        }
        if (gamepad2.dpadDownWasPressed()) {
            scheduler.schedule(new SetIndexerStateCommand(indexer, Indexer.State.REVERSING));
        }
        if (gamepad2.bWasPressed()) {
            scheduler.schedule(new SetIndexerStateCommand(indexer, Indexer.State.IDLE));
        }

        // Flywheel (gamepad2). X spins up to shooting RPM, Y stops it.
        if (gamepad2.xWasPressed()) {
            scheduler.schedule(new SetFlywheelRpmCommand(flywheel, Constants.Flywheel.SHOOT_RPM));
        }
        if (gamepad2.yWasPressed()) {
            scheduler.schedule(new SetFlywheelRpmCommand(flywheel, 0));
        }

        scheduler.run();

        telemetry.addData("Mode", fieldCentric ? "Field-centric" : "Robot-centric");
        telemetry.addData("Heading (deg)", Math.toDegrees(drivebase.getHeading()));
        telemetry.addData("Intake", intake.getState());
        telemetry.addData("Indexer", indexer.getState());
        telemetry.addData("Flywheel target", flywheel.getTargetRpm());
        telemetry.addData("Flywheel actual", "%.0f rpm", flywheel.getCurrentRpm());
        telemetry.addData("At speed", flywheel.atTargetRpm());
        telemetry.update();
    }

    @Override
    public void stop(){
        scheduler.cancelAll();
        drivebase.stop();
        flywheel.stop();
    }
}
