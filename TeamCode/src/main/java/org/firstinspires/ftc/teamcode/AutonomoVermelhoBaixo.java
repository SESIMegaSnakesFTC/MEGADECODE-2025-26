package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "VERMELHO BAIXO")
public class AutonomoVermelhoBaixo extends OpMode {

    private DcMotorEx baseShooter;
    private DcMotorEx feeder;
    private Follower follower;
    private Paths paths;
    private Timer pathTimer;
    private PathState pathState;

    private static final int POSICAO_SHOOT = 300;
    private static final int TOLERANCIA = 10;

    public enum PathState {

        RESET_MOTOR,
        POS_ZERO_BASE,
        POS_SHOOT_BASE,
        AJUSTANDO_FEEDING,
        FEEDING,
        AJUSTE_EMPURRAR,
        EMPURRAR,
        AJUSTE_PAREDE,
        DESCE_SHOOT_1,
        FEEDING_BASE,
        DESCE_SHOOT_2,
        DONE
    }

    public static class Paths {
        public PathChain AJUSTANDOFEEDING;
        public PathChain FEEDING;
        public PathChain AJUSTEEMPURRAR;
        public PathChain EMPURRAR;
        public PathChain AJUSTEPAREDE;
        public PathChain DESCESHOOT1;
        public PathChain FEEDINGBASE;
        public PathChain DESCESHOOT2;

        public Paths(Follower follower) {
            AJUSTANDOFEEDING = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(56.000, 9.000),
                            new Pose(40.000, 65.800)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(90),
                            Math.toRadians(180))
                    .build();

            FEEDING = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(40.000, 65.800),
                            new Pose(7.750, 65.800)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(180))
                    .build();

            AJUSTEEMPURRAR = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(7.750, 65.800),
                            new Pose(23.430, 52.500)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(151),
                            Math.toRadians(151))
                    .build();

            EMPURRAR = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(23.430, 52.500),
                            new Pose(12.343, 17.500)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(140),
                            Math.toRadians(140))
                    .build();

            AJUSTEPAREDE = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(12.343, 17.500),
                            new Pose(12.343, 16.000)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(180))
                    .build();

            DESCESHOOT1 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(12.343, 16.000),
                            new Pose(56.000, 16.000)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(180))
                    .build();

            FEEDINGBASE = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(56.000, 16.000),
                            new Pose(0.000, 16.000)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(180))
                    .build();

            DESCESHOOT2 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(0.000, 16.000),
                            new Pose(56.000, 16.000)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(180))
                    .build();
        }
    }

    @Override
    public void init() {
        feeder = hardwareMap.get(DcMotorEx.class, "feeder");

        baseShooter = hardwareMap.get(DcMotorEx.class, "baseShooter");
        baseShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        baseShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        baseShooter.setDirection(DcMotorSimple.Direction.FORWARD);

        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower);

        pathTimer = new Timer();
        pathState = PathState.RESET_MOTOR;

        // Posição inicial baseada no primeiro ponto do primeiro caminho
        follower.setPose(new Pose(56.000, 9.000, Math.toRadians(90)));
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

            case RESET_MOTOR:
                baseShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                baseShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                setPathState(PathState.POS_ZERO_BASE);
                break;

            case POS_ZERO_BASE:
                baseShooter.setTargetPosition(0);
                baseShooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                baseShooter.setPower(0.4);
                setPathState(PathState.POS_SHOOT_BASE);
                break;

            case POS_SHOOT_BASE:
                if(Math.abs(baseShooter.getCurrentPosition()) <= TOLERANCIA){
                    baseShooter.setTargetPosition(POSICAO_SHOOT);
                    baseShooter.setPower(0.5);
                    setPathState(PathState.AJUSTANDO_FEEDING);
                }
                break;

            case AJUSTANDO_FEEDING:
                follower.setMaxPower(0.7);
                follower.followPath(paths.AJUSTANDOFEEDING, true);
                setPathState(PathState.FEEDING);
                break;

            case FEEDING:
                if (!follower.isBusy()) {
                    feeder.setPower(-1);

                    follower.setMaxPower(0.3);
                    follower.followPath(paths.FEEDING, true);
                    setPathState(PathState.AJUSTE_EMPURRAR);
                }
                break;

            case AJUSTE_EMPURRAR:
                if (!follower.isBusy()) {
                    feeder.setPower(-1);

                    follower.setMaxPower(0.5);
                    follower.followPath(paths.AJUSTEEMPURRAR, true);
                    setPathState(PathState.EMPURRAR);
                    feeder.setPower(0);
                }

                break;

            case EMPURRAR:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.5);
                    follower.followPath(paths.EMPURRAR, true);
                    setPathState(PathState.AJUSTE_PAREDE);
                }
                break;

            case AJUSTE_PAREDE:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.5);
                    follower.followPath(paths.AJUSTEPAREDE, true);
                    setPathState(PathState.DESCE_SHOOT_1);
                }
                break;

            case DESCE_SHOOT_1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {

                    follower.setMaxPower(1);
                    follower.followPath(paths.DESCESHOOT1, true);
                    setPathState(PathState.FEEDING_BASE);
                }
                break;

            case FEEDING_BASE:
                if (!follower.isBusy()) {
                    feeder.setPower(-1); // Ativa o feeder
                    follower.setMaxPower(0.6);
                    follower.followPath(paths.FEEDINGBASE, true);
                    setPathState(PathState.DESCE_SHOOT_2);
                }
                break;

            case DESCE_SHOOT_2:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(paths.DESCESHOOT2, true);
                    setPathState(PathState.DONE);
                }
                break;

            case DONE:
                // Para o feeder quando terminar
                feeder.setPower(0);
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
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Tempo Estado", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Busy", follower.isBusy());
    }
}