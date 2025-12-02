package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Autonomo TESTE")
public class AutonomoP2Vermelho extends OpMode {

    private DcMotorEx feeder;
    private Follower follower;
    private Paths paths;
    private Timer pathTimer;

    private PathState pathState;
    public enum PathState {
        DESCER_SHOOT,
        AJUSTE_FEED,
        FEEDING,
        VOLTA_SHOOT,
        DONE
    }

    public static class Paths {

        public PathChain DesceShoot;
        public PathChain AjusteFeed;
        public PathChain Feeding;
        public PathChain VoltaShoot;

        public Paths(Follower follower) {

            DesceShoot = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(124.155, 123.187),
                            new Pose(90, 89.5)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(37),
                            Math.toRadians(0)
                    )
                    .build();

            AjusteFeed = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(104.067, 103.099),
                            new Pose(104.067, 90)
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            Feeding = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(104.067, 84.000),
                            new Pose(123, 84.000)
                    ))
                    .setTangentHeadingInterpolation()
                    .setVelocityConstraint(0.5)
                    .build();

            VoltaShoot = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(129.500, 84.000),
                            new Pose(104.067, 103.099)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(0)
                    )
                    .build();
        }
    }

    @Override
    public void init() {

        feeder = hardwareMap.get(DcMotorEx.class, "feeder");

        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower);

        pathTimer = new Timer();
        pathState = PathState.DESCER_SHOOT;

        follower.setPose(
                new Pose(124.155, 123.187, Math.toRadians(37))
        );
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
    }

    private void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    private void statePathUpdate() {

        switch (pathState) {

            case DESCER_SHOOT:
                follower.setMaxPower(1);
                follower.followPath(paths.DesceShoot, true);
                setPathState(PathState.AJUSTE_FEED);
                break;

            case AJUSTE_FEED:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.3);
                    follower.followPath(paths.AjusteFeed, true);
                    setPathState(PathState.FEEDING);
                }
                break;

            case FEEDING:
                if (!follower.isBusy()) {
                    feeder.setPower(-1);
                    follower.setMaxPower(0.3);
                    follower.followPath(paths.Feeding, true);
                    setPathState(PathState.VOLTA_SHOOT);
                }
                break;

            case VOLTA_SHOOT:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    feeder.setPower(-1);
                    follower.followPath(paths.VoltaShoot, true);
                    setPathState(PathState.DONE);
                }
                break;

            case DONE:
                // fim
                break;
        }
    }


    @Override
    public void loop() {

        follower.update();
        statePathUpdate();

        telemetry.addData("Estado", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Tempo Estado", pathTimer.getElapsedTimeSeconds());
    }
}
