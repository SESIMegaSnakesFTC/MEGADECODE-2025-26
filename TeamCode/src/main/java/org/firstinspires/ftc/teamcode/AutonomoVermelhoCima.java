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

@Autonomous(name = "VERMELHO CIMA")
public class AutonomoVermelhoCima extends OpMode {
    private DcMotorEx feeder;
    private Follower follower;
    private Paths paths;
    private Timer pathTimer;

    // Constantes de tempo (em segundos)
    private static final double TEMPO_FEED_MEIO = 2.0; // Tempo total do FEEDMEIO
    private static final double TEMPO_FEED_CIMA = 2.0; // Tempo total do FEEDCIMA
    private static final double DELAY_APOS_FEED = 0.5; // Tempo que o feeder fica ligado após o movimento
    private static final double ESPERA_SHOOT = 4.0; // Tempo de espera nos caminhos de shoot

    public enum PathState {
        DESCERSHOOT1,
        ESPERA_SHOOT1,
        AJUSTEFEEDMEIO,
        FEEDMEIO_INICIAR,
        FEEDMEIO_EM_ANDAMENTO,
        FEEDMEIO_FINALIZAR,
        AJUSTEVOLTA,
        VOLTASHOOT2,
        ESPERA_SHOOT2,
        AJUSTEFEEDCIMA,
        FEEDCIMA_INICIAR,
        FEEDCIMA_EM_ANDAMENTO,
        FEEDCIMA_FINALIZAR,
        VOLTASHOOT3,
        ESPERA_SHOOT3,
        DONE
    }

    private PathState pathState;

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
                            Math.toRadians(126),
                            Math.toRadians(0))
                    .build();

            AJUSTEFEEDMEIO = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(90, 89.5),
                            new Pose(93, 60)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(0))
                    .build();

            FEEDMEIO = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(93, 60),
                            new Pose(125, 59.7)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(0))
                    .build();

            AJUSTEVOLTA = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(125, 59.7),
                            new Pose(90, 59.7)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(0))
                    .build();

            VOLTASHOOT2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(90, 59.7),
                            new Pose(90, 89.5)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(0))
                    .build();

            AJUSTEFEEDCIMA = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(90, 89.5),
                            new Pose(98, 84.2)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(0))
                    .build();

            FEEDCIMA = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(103, 84.2),
                            new Pose(127, 84.2)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(0))
                    .build();

            VOLTASHOOT3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(126, 84.2),
                            new Pose(90, 89.5)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(0))
                    .build();
        }
    }

    @Override
    public void init() {
        feeder = hardwareMap.get(DcMotorEx.class, "feeder");

        // Configura o feeder
        feeder.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        feeder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower);

        pathTimer = new Timer();
        pathState = PathState.DESCERSHOOT1;

        follower.setPose(
                new Pose(124.155,
                        123.187,
                        Math.toRadians(126))
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
            case DESCERSHOOT1:
                follower.setMaxPower(1.0);
                follower.followPath(paths.DESCERSHOOT1, true);
                setPathState(PathState.ESPERA_SHOOT1);
                break;

            case ESPERA_SHOOT1:
                // Espera 4 segundos parado no final do DESCERSHOOT1
                if (pathTimer.getElapsedTimeSeconds() >= ESPERA_SHOOT) {
                    setPathState(PathState.AJUSTEFEEDMEIO);
                }
                break;

            case AJUSTEFEEDMEIO:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.46);
                    follower.followPath(paths.AJUSTEFEEDMEIO, true);
                    setPathState(PathState.FEEDMEIO_INICIAR);
                }
                break;

            case FEEDMEIO_INICIAR:
                if (!follower.isBusy()) {
                    // LIGA O FEEDER
                    feeder.setPower(-1);

                    // Inicia o movimento com velocidade reduzida
                    follower.setMaxPower(0.25);
                    follower.followPath(paths.FEEDMEIO, true);

                    // Marca o tempo de início
                    setPathState(PathState.FEEDMEIO_EM_ANDAMENTO);
                }
                break;

            case FEEDMEIO_EM_ANDAMENTO:
                // Verifica se o tempo total do movimento já passou
                if (pathTimer.getElapsedTimeSeconds() >= TEMPO_FEED_MEIO) {
                    // Para o movimento (se ainda estiver em andamento)
                    if (follower.isBusy()) {
                        follower.breakFollowing();
                    }

                    // Mantém o feeder ligado por mais um tempo
                    setPathState(PathState.FEEDMEIO_FINALIZAR);
                }
                // Se o movimento terminar antes do tempo, também vai para o próximo estado
                else if (!follower.isBusy()) {
                    setPathState(PathState.FEEDMEIO_FINALIZAR);
                }
                break;

            case FEEDMEIO_FINALIZAR:
                // Mantém o feeder ligado por um tempo extra
                if (pathTimer.getElapsedTimeSeconds() >= DELAY_APOS_FEED) {
                    // DESLIGA O FEEDER
                    feeder.setPower(0);
                    setPathState(PathState.AJUSTEVOLTA);
                }
                break;

            case AJUSTEVOLTA:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.3);
                    follower.followPath(paths.AJUSTEVOLTA, true);
                    setPathState(PathState.VOLTASHOOT2);
                }
                break;

            case VOLTASHOOT2:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(paths.VOLTASHOOT2, true);
                    setPathState(PathState.ESPERA_SHOOT2);
                }
                break;

            case ESPERA_SHOOT2:
                // Espera 4 segundos parado no final do VOLTASHOOT2
                if (pathTimer.getElapsedTimeSeconds() >= ESPERA_SHOOT) {
                    setPathState(PathState.AJUSTEFEEDCIMA);
                }
                break;

            case AJUSTEFEEDCIMA:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(paths.AJUSTEFEEDCIMA, true);
                    setPathState(PathState.FEEDCIMA_INICIAR);
                }
                break;

            case FEEDCIMA_INICIAR:
                if (!follower.isBusy()) {
                    // LIGA O FEEDER
                    feeder.setPower(-1);

                    // Inicia o movimento com velocidade reduzida
                    follower.setMaxPower(0.3);
                    follower.followPath(paths.FEEDCIMA, true);

                    // Marca o tempo de início
                    setPathState(PathState.FEEDCIMA_EM_ANDAMENTO);
                }
                break;

            case FEEDCIMA_EM_ANDAMENTO:
                // Verifica se o tempo total do movimento já passou
                if (pathTimer.getElapsedTimeSeconds() >= TEMPO_FEED_CIMA) {
                    // Para o movimento (se ainda estiver em andamento)
                    if (follower.isBusy()) {
                        follower.breakFollowing();
                    }

                    // Mantém o feeder ligado por mais um tempo
                    setPathState(PathState.FEEDCIMA_FINALIZAR);
                }
                // Se o movimento terminar antes do tempo, também vai para o próximo estado
                else if (!follower.isBusy()) {
                    setPathState(PathState.FEEDCIMA_FINALIZAR);
                }
                break;

            case FEEDCIMA_FINALIZAR:
                // Mantém o feeder ligado por um tempo extra
                if (pathTimer.getElapsedTimeSeconds() >= DELAY_APOS_FEED) {
                    // DESLIGA O FEEDER
                    feeder.setPower(0);
                    setPathState(PathState.VOLTASHOOT3);
                }
                break;

            case VOLTASHOOT3:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(paths.VOLTASHOOT3, true);
                    setPathState(PathState.ESPERA_SHOOT3);
                }
                break;

            case ESPERA_SHOOT3:
                // Espera 4 segundos parado no final do VOLTASHOOT3
                if (pathTimer.getElapsedTimeSeconds() >= ESPERA_SHOOT) {
                    setPathState(PathState.DONE);
                }
                break;

            case DONE:
                // Garante que o feeder está desligado
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
        telemetry.addData("Feeder Power", feeder.getPower());
        telemetry.addData("Tempo Total", pathTimer.getElapsedTimeSeconds());
    }
}