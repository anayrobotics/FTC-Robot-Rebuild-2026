package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Autonomous(name="Preload Auto Red")
public class PreloadAuto extends OpMode {
    Drivebase drivebase;
    Intake intake;
    Indexer indexer;
    Turret turret;

    Flywheel flywheel;

    Hardware hardware;

    Limelight limelight;

    @Override
    public void init(){
        hardware = new Hardware();
        hardware.initDrive(hardwareMap);
        hardware.initIntake(hardwareMap);
        hardware.initIndexer(hardwareMap);
        hardware.initFlywheel(hardwareMap);
        hardware.initTurret(hardwareMap);
        hardware.initLimelight(hardwareMap);
        hardware.initImu(hardwareMap);

        drivebase = new Drivebase(hardware);
        intake = new Intake(hardware);
        indexer = new Indexer(hardware);
        limelight = new Limelight(hardware);
        turret = new Turret(hardware, limelight);
        flywheel = new Flywheel(hardware);
    }

    @Override
    public void start(){

    }
}
