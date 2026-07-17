package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Example PedroPathing autonomous.
 *
 * <p>Drives a simple two-leg path off the start line using the follower built
 * in {@link Constants}. It is written as an iterative {@link OpMode} with a
 * small state machine — the idiomatic PedroPathing pattern — so
 * {@link Follower#update()} is called every loop and paths advance as each one
 * finishes.
 *
 * <p>All poses are in PedroPathing field coordinates (inches, heading in
 * radians). Replace the poses below with your real field geometry.
 */
@Autonomous(name = "Pedro Auto Example", group = "PedroPathing")
public class PedroAutoExample extends OpMode {

    private Follower follower;
    private int pathState;

    // Where the robot physically starts on the field. Also used as the follower's
    // pose estimate origin, so set this to match your real starting position.
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose firstTarget = new Pose(24, 0, Math.toRadians(0));    // forward 24"
    private final Pose secondTarget = new Pose(24, 24, Math.toRadians(90)); // strafe + turn

    private PathChain toFirst;
    private PathChain toSecond;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
        telemetry.addLine("PedroPathing ready. Press play.");
        telemetry.update();
    }

    private void buildPaths() {
        toFirst = follower.pathBuilder()
                .addPath(new BezierLine(startPose, firstTarget))
                .setLinearHeadingInterpolation(startPose.getHeading(), firstTarget.getHeading())
                .build();

        toSecond = follower.pathBuilder()
                .addPath(new BezierLine(firstTarget, secondTarget))
                .setLinearHeadingInterpolation(firstTarget.getHeading(), secondTarget.getHeading())
                .build();
    }

    @Override
    public void start() {
        setPathState(0);
    }

    @Override
    public void loop() {
        // Must run every loop for the follower to track and drive.
        follower.update();
        runStateMachine();

        Pose pose = follower.getPose();
        telemetry.addData("state", pathState);
        telemetry.addData("busy", follower.isBusy());
        telemetry.addData("x", pose.getX());
        telemetry.addData("y", pose.getY());
        telemetry.addData("heading (deg)", Math.toDegrees(pose.getHeading()));
        telemetry.update();
    }

    // Advances to the next path once the follower finishes the current one.
    private void runStateMachine() {
        switch (pathState) {
            case 0:
                follower.followPath(toFirst);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(toSecond);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    setPathState(-1); // done
                }
                break;
            default:
                // -1: finished, nothing to do.
                break;
        }
    }

    private void setPathState(int state) {
        pathState = state;
    }
}
