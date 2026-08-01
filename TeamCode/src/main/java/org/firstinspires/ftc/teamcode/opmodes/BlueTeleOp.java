package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;

/**
 * Blue alliance single-driver TeleOp. All the behaviour and the control map
 * live in {@link MecanumTeleOp}; this only says which goal to shoot at.
 */
@TeleOp(name = "TeleOp 67 — BLUE", group = "Drive")
public class BlueTeleOp extends MecanumTeleOp {
    @Override
    protected int goalTagId() {
        return Constants.Vision.BLUE_GOAL_TAG;
    }
}
