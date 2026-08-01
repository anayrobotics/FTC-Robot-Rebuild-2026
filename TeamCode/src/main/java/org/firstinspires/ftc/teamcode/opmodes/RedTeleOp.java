package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;

/**
 * Red alliance single-driver TeleOp. All the behaviour and the control map
 * live in {@link MecanumTeleOp}; this only says which goal to shoot at.
 */
@TeleOp(name = "TeleOp 67 — RED", group = "Drive")
public class RedTeleOp extends MecanumTeleOp {
    @Override
    protected int goalTagId() {
        return Constants.Vision.RED_GOAL_TAG;
    }
}
