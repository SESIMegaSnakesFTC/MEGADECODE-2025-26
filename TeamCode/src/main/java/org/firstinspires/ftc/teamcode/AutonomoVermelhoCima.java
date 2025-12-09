package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;



@Autonomous(name = "VERMELHO CIMA")
public class AutonomoVermelhoCima extends OpMode {

    private DcMotorEx feeder;
    private Follower follower;
    private Paths paths;
    private Timer pathTimer;
    private PathState pathState;
    public enum PathState {
        DESCER_SHOOT1,
        AJUSTE_FEED_MEIO,
        FEED_MEIO,
        AJUSTE_VOLTA,
        VOLTA_SHOOT2,
        AJUSTE_FEED_CIMA,
        FEED_CIMA,
        VOLTA_SHOOT3,
        DONE
    }

    public static class Paths {

        public PathChain DESCERSHOOT1;
        public PathChain AJUSTEFEEDMEIO;
        public PathChain FEEDMEIO;
        public PathChain AJUSTEVOLTA;
        public PathChain VOLTASHOOT2;
        public PathChain AJUSTEFEEDCIMA;
        public PathChain FEEDCIMA;
        public PathChain VOLTASHOOT3;

        public Paths(Follower follower) {

            DESCERSHOOT1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(124.155, 123.187),
                            new Pose(90, 89.5)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(43),
                            Math.toRadians(0)
                    )
                    .build();

            AJUSTEFEEDMEIO = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(90, 89.5),
                            new Pose(93, 60)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(0)
                    )
                    .build();

            FEEDMEIO = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(93, 60),
                            new Pose(125, 59.7)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(0)
                    )
                    .build();

            AJUSTEVOLTA = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(125, 59.7),
                            new Pose(90, 59.7)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(0)
                    )
                    .build();

            VOLTASHOOT2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(90, 59.7),
                            new Pose(90, 89.5)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(0)
                    )
                    .build();

            AJUSTEFEEDCIMA = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(90, 89.5),
                            new Pose(98, 84.2)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(0)
                    )
                    .build();

            FEEDCIMA = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(103, 84.2),
                            new Pose(127, 84.2)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(0)
                    )
                    .build();

            VOLTASHOOT3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(126, 84.2),
                            new Pose(90, 89.5)
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
        pathState = PathState.DESCER_SHOOT1;

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

            case DESCER_SHOOT1:
                follower.setMaxPower(1);
                follower.followPath(paths.DESCERSHOOT1, true);
                setPathState(PathState.AJUSTE_FEED_MEIO);
                break;

            case AJUSTE_FEED_MEIO:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.46);
                    follower.followPath(paths.AJUSTEFEEDMEIO, true);
                    setPathState(PathState.FEED_MEIO);
                }
                break;

            case FEED_MEIO:
                if (!follower.isBusy()) {
                    feeder.setPower(-1);

                    follower.setMaxPower(0.25);
                    follower.followPath(paths.FEEDMEIO, true);
                    setPathState(PathState.AJUSTE_VOLTA);
                }
                break;

            case AJUSTE_VOLTA:
                if (!follower.isBusy()) {
                    feeder.setPower(-1);

                    follower.setMaxPower(0.3);
                    follower.followPath(paths.AJUSTEVOLTA, true);
                    setPathState(PathState.VOLTA_SHOOT2);

                    feeder.setPower(0);
                }
                break;

            case VOLTA_SHOOT2:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(paths.VOLTASHOOT2, true);
                    setPathState(PathState.AJUSTE_FEED_CIMA);
                }
                break;

            case AJUSTE_FEED_CIMA:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(paths.AJUSTEFEEDCIMA, true);
                    setPathState(PathState.FEED_CIMA);
                }
                break;

            case FEED_CIMA:
                if (!follower.isBusy()) {
                    feeder.setPower(-1);

                    follower.setMaxPower(0.3);
                    follower.followPath(paths.FEEDCIMA, true);
                    setPathState(PathState.VOLTA_SHOOT3);
                }
                break;

            case VOLTA_SHOOT3:
                if (!follower.isBusy()) {
                    feeder.setPower(-1);

                    follower.setMaxPower(1);
                    follower.followPath(paths.VOLTASHOOT3, true);
                    setPathState(PathState.DONE);

                    feeder.setPower(0);
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
