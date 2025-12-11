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

@Autonomous(name = "VERMELHO BAIXO")
public class AutonomoVermelhoBaixo extends OpMode {
    private DcMotorEx feeder;
    private Follower follower;
    private Paths paths;
    private Timer pathTimer;

    // Constantes de tempo (em segundos)
    private static final double TEMPO_FEED_MEIO = 2.0; // Tempo total do FEEDMEIO
    private static final double TEMPO_FEED_BAIXO = 2.0; // Tempo total do FEEDBAIXO
    private static final double DELAY_APOS_FEED = 0.5; // Tempo que o feeder fica ligado após o movimento
    private static final double ESPERA_SHOOT = 4.0; // Tempo de espera nos caminhos de shoot

    public enum PathState {
        IDAFRENTEINICIAL,
        IDALAUNCHZONE1,
        CAMINHOSHOOT1,
        ESPERA_SHOOT1,
        IDAFEEDMEIO,
        FEEDMEIO_INICIAR,
        FEEDMEIO_EM_ANDAMENTO,
        FEEDMEIO_FINALIZAR,
        AJUSTECAMINHOSHOOT2,
        CAMINHOSHOOT2,
        ESPERA_SHOOT2,
        IDAFEEDBAIXO,
        FEEDBAIXO_INICIAR,
        FEEDBAIXO_EM_ANDAMENTO,
        FEEDBAIXO_FINALIZAR,
        CAMINHOSHOOT3,
        ESPERA_SHOOT3,
        DONE
    }

    private PathState pathState;

    public static class Paths {
        public PathChain IDAFRENTEINICIAL;
        public PathChain IDALAUNCHZONE1;
        public PathChain CAMINHOSHOOT1;
        public PathChain IDAFEEDMEIO;
        public PathChain FEEDMEIO;
        public PathChain AJUSTECAMINHOSHOOT2;
        public PathChain CAMINHOSHOOT2;
        public PathChain IDAFEEDBAIXO;
        public PathChain FEEDBAIXO;
        public PathChain CAMINHOSHOOT3;

        public Paths(Follower follower) {
            IDAFRENTEINICIAL = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(56.000, 8.000),
                            new Pose(56.000, 24.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            IDALAUNCHZONE1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(56.000, 24.000),
                            new Pose(91.300, 90.100))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            CAMINHOSHOOT1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(91.300, 90.100),
                            new Pose(101.300, 100.100))
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(126),
                            Math.toRadians(180))
                    .build();

            IDAFEEDMEIO = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(101.300, 100.100),
                            new Pose(46.000, 58.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            FEEDMEIO = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(46.000, 58.000),
                            new Pose(11.000, 60.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            AJUSTECAMINHOSHOOT2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(11.000, 60.000),
                            new Pose(33.000, 60.000))
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(180))
                    .build();

            CAMINHOSHOOT2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(33.000, 60.000),
                            new Pose(101.300, 100.100))
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(180))
                    .build();

            IDAFEEDBAIXO = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(101.300, 100.100),
                            new Pose(46.000, 36.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            FEEDBAIXO = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(46.000, 36.000),
                            new Pose(11.000, 36.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            CAMINHOSHOOT3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(11.000, 36.000),
                            new Pose(101.300, 100.100))
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(180))
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
        pathState = PathState.IDAFRENTEINICIAL;

        follower.setPose(
                new Pose(56.000,
                        8.000,
                        Math.toRadians(90))
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
            case IDAFRENTEINICIAL:
                follower.setMaxPower(1.0);
                follower.followPath(paths.IDAFRENTEINICIAL, true);
                setPathState(PathState.IDALAUNCHZONE1);
                break;

            case IDALAUNCHZONE1:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1.0);
                    follower.followPath(paths.IDALAUNCHZONE1, true);
                    setPathState(PathState.CAMINHOSHOOT1);
                }
                break;

            case CAMINHOSHOOT1:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1.0);
                    follower.followPath(paths.CAMINHOSHOOT1, true);
                    setPathState(PathState.ESPERA_SHOOT1);
                }
                break;

            case ESPERA_SHOOT1:
                // Espera 4 segundos parado no final do CAMINHOSHOOT1
                if (pathTimer.getElapsedTimeSeconds() >= ESPERA_SHOOT) {
                    setPathState(PathState.IDAFEEDMEIO);
                }
                break;

            case IDAFEEDMEIO:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1.0);
                    follower.followPath(paths.IDAFEEDMEIO, true);
                    setPathState(PathState.FEEDMEIO_INICIAR);
                }
                break;

            case FEEDMEIO_INICIAR:
                if (!follower.isBusy()) {
                    // LIGA O FEEDER
                    feeder.setPower(-1);

                    // Inicia o movimento com velocidade reduzida
                    follower.setMaxPower(0.5);
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
                    setPathState(PathState.AJUSTECAMINHOSHOOT2);
                }
                break;

            case AJUSTECAMINHOSHOOT2:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.7);
                    follower.followPath(paths.AJUSTECAMINHOSHOOT2, true);
                    setPathState(PathState.CAMINHOSHOOT2);
                }
                break;

            case CAMINHOSHOOT2:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1.0);
                    follower.followPath(paths.CAMINHOSHOOT2, true);
                    setPathState(PathState.ESPERA_SHOOT2);
                }
                break;

            case ESPERA_SHOOT2:
                // Espera 4 segundos parado no final do CAMINHOSHOOT2
                if (pathTimer.getElapsedTimeSeconds() >= ESPERA_SHOOT) {
                    setPathState(PathState.IDAFEEDBAIXO);
                }
                break;

            case IDAFEEDBAIXO:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1.0);
                    follower.followPath(paths.IDAFEEDBAIXO, true);
                    setPathState(PathState.FEEDBAIXO_INICIAR);
                }
                break;

            case FEEDBAIXO_INICIAR:
                if (!follower.isBusy()) {
                    // LIGA O FEEDER
                    feeder.setPower(-1);

                    // Inicia o movimento com velocidade reduzida
                    follower.setMaxPower(0.5);
                    follower.followPath(paths.FEEDBAIXO, true);

                    // Marca o tempo de início
                    setPathState(PathState.FEEDBAIXO_EM_ANDAMENTO);
                }
                break;

            case FEEDBAIXO_EM_ANDAMENTO:
                // Verifica se o tempo total do movimento já passou
                if (pathTimer.getElapsedTimeSeconds() >= TEMPO_FEED_BAIXO) {
                    // Para o movimento (se ainda estiver em andamento)
                    if (follower.isBusy()) {
                        follower.breakFollowing();
                    }

                    // Mantém o feeder ligado por mais um tempo
                    setPathState(PathState.FEEDBAIXO_FINALIZAR);
                }
                // Se o movimento terminar antes do tempo, também vai para o próximo estado
                else if (!follower.isBusy()) {
                    setPathState(PathState.FEEDBAIXO_FINALIZAR);
                }
                break;

            case FEEDBAIXO_FINALIZAR:
                // Mantém o feeder ligado por um tempo extra
                if (pathTimer.getElapsedTimeSeconds() >= DELAY_APOS_FEED) {
                    // DESLIGA O FEEDER
                    feeder.setPower(0);
                    setPathState(PathState.CAMINHOSHOOT3);
                }
                break;

            case CAMINHOSHOOT3:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1.0);
                    follower.followPath(paths.CAMINHOSHOOT3, true);
                    setPathState(PathState.ESPERA_SHOOT3);
                }
                break;

            case ESPERA_SHOOT3:
                // Espera 4 segundos parado no final do CAMINHOSHOOT3
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