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
    // Hardware
    private DcMotorEx feeder;
    private Follower follower;

    // Controladores
    private Paths paths;
    private Timer pathTimer;
    private ShooterController shooterController; // NOVO

    // Constantes de tempo (em segundos)
    private static final double TEMPO_FEED_MEIO = 2.0;
    private static final double TEMPO_FEED_CIMA = 2.0;
    private static final double DELAY_APOS_FEED = 0.5;
    private static final double ESPERA_SHOOT = 8.0; // Aumentado para dar tempo do ciclo completo

    public enum PathState {
        // Estados originais
        DESCERSHOOT1,
        ESPERA_SHOOT1,           // Agora gerencia ciclo de shoot
        AJUSTEFEEDMEIO,
        FEEDMEIO_INICIAR,
        FEEDMEIO_EM_ANDAMENTO,
        FEEDMEIO_FINALIZAR,
        AJUSTEVOLTA,
        VOLTASHOOT2,
        ESPERA_SHOOT2,           // Agora gerencia ciclo de shoot
        AJUSTEFEEDCIMA,
        FEEDCIMA_INICIAR,
        FEEDCIMA_EM_ANDAMENTO,
        FEEDCIMA_FINALIZAR,
        VOLTASHOOT3,
        ESPERA_SHOOT3,           // Agora gerencia ciclo de shoot
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
                            new Pose(102, 101)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(126),
                            Math.toRadians(0))
                    .build();

            AJUSTEFEEDMEIO = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(102, 101),
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
        // Inicializa feeder
        feeder = hardwareMap.get(DcMotorEx.class, "feeder");
        feeder.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        feeder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Inicializa shooter controller (NOVO)
        shooterController = new ShooterController();
        shooterController.init(hardwareMap, feeder);

        // Inicializa follower e paths
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
                // Liga shooter ANTES de começar o movimento
                shooterController.iniciarCiclo3Bolinhas();

                follower.setMaxPower(1.0);
                follower.followPath(paths.DESCERSHOOT1, true);
                setPathState(PathState.ESPERA_SHOOT1);
                break;

            case ESPERA_SHOOT1:
                // Atualiza o controlador de shoot
                shooterController.update();

                // FASE 1: Aguarda shooter acelerar (primeiros 2 segundos)
                if (pathTimer.getElapsedTimeSeconds() < 2.0) {
                    // Apenas espera shooter acelerar
                    telemetry.addLine("Acelerando shooter...");
                }
                // FASE 2: Começa a atirar quando shooter estiver pronto
                else if (shooterController.isReadyToShoot()) {
                    shooterController.comecarATirar();
                    telemetry.addLine("Atirando bolinhas...");
                }
                // FASE 3: Quando ciclo terminar, vai para próximo estado
                else if (shooterController.isIdle()) {
                    setPathState(PathState.AJUSTEFEEDMEIO);
                }
                // FASE 4: Timeout de segurança
                else if (pathTimer.getElapsedTimeSeconds() >= ESPERA_SHOOT) {
                    shooterController.emergencyStop();
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
                    // LIGA O FEEDER (para coleta)
                    feeder.setPower(-1);

                    follower.setMaxPower(0.25);
                    follower.followPath(paths.FEEDMEIO, true);

                    setPathState(PathState.FEEDMEIO_EM_ANDAMENTO);
                }
                break;

            case FEEDMEIO_EM_ANDAMENTO:
                if (pathTimer.getElapsedTimeSeconds() >= TEMPO_FEED_MEIO) {
                    if (follower.isBusy()) {
                        follower.breakFollowing();
                    }
                    setPathState(PathState.FEEDMEIO_FINALIZAR);
                } else if (!follower.isBusy()) {
                    setPathState(PathState.FEEDMEIO_FINALIZAR);
                }
                break;

            case FEEDMEIO_FINALIZAR:
                if (pathTimer.getElapsedTimeSeconds() >= DELAY_APOS_FEED) {
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
                    // Liga shooter para próximo ciclo
                    shooterController.iniciarCiclo3Bolinhas();

                    follower.setMaxPower(1);
                    follower.followPath(paths.VOLTASHOOT2, true);
                    setPathState(PathState.ESPERA_SHOOT2);
                }
                break;

            case ESPERA_SHOOT2:
                // Mesma lógica do ESPERA_SHOOT1
                shooterController.update();

                if (pathTimer.getElapsedTimeSeconds() < 2.0) {
                    // Aguarda shooter acelerar
                }
                else if (shooterController.isReadyToShoot()) {
                    shooterController.comecarATirar();
                }
                else if (shooterController.isIdle()) {
                    setPathState(PathState.AJUSTEFEEDCIMA);
                }
                else if (pathTimer.getElapsedTimeSeconds() >= ESPERA_SHOOT) {
                    shooterController.emergencyStop();
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
                    feeder.setPower(-1);
                    follower.setMaxPower(0.3);
                    follower.followPath(paths.FEEDCIMA, true);
                    setPathState(PathState.FEEDCIMA_EM_ANDAMENTO);
                }
                break;

            case FEEDCIMA_EM_ANDAMENTO:
                if (pathTimer.getElapsedTimeSeconds() >= TEMPO_FEED_CIMA) {
                    if (follower.isBusy()) {
                        follower.breakFollowing();
                    }
                    setPathState(PathState.FEEDCIMA_FINALIZAR);
                } else if (!follower.isBusy()) {
                    setPathState(PathState.FEEDCIMA_FINALIZAR);
                }
                break;

            case FEEDCIMA_FINALIZAR:
                if (pathTimer.getElapsedTimeSeconds() >= DELAY_APOS_FEED) {
                    feeder.setPower(0);
                    setPathState(PathState.VOLTASHOOT3);
                }
                break;

            case VOLTASHOOT3:
                if (!follower.isBusy()) {
                    // Liga shooter para último ciclo
                    shooterController.iniciarCiclo3Bolinhas();

                    follower.setMaxPower(1);
                    follower.followPath(paths.VOLTASHOOT3, true);
                    setPathState(PathState.ESPERA_SHOOT3);
                }
                break;

            case ESPERA_SHOOT3:
                // Mesma lógica dos outros shoots
                shooterController.update();

                if (pathTimer.getElapsedTimeSeconds() < 2.0) {
                    // Aguarda shooter acelerar
                }
                else if (shooterController.isReadyToShoot()) {
                    shooterController.comecarATirar();
                }
                else if (shooterController.isIdle()) {
                    setPathState(PathState.DONE);
                }
                else if (pathTimer.getElapsedTimeSeconds() >= ESPERA_SHOOT) {
                    shooterController.emergencyStop();
                    setPathState(PathState.DONE);
                }
                break;

            case DONE:
                // Garante que tudo está desligado
                feeder.setPower(0);
                shooterController.emergencyStop();
                break;
        }
    }

    @Override
    public void loop() {
        // Atualiza follower (movimento)
        follower.update();

        // Atualiza máquina de estados
        statePathUpdate();

        // Telemetria detalhada
        telemetry.addData("Estado", pathState);
        telemetry.addData("Estado Shoot", shooterController.getEstadoAtual());
        telemetry.addData("Bolinhas Atiradas", shooterController.getBolinhasAtiradas());
        telemetry.addData("Tempo Estado Shoot", shooterController.getTempoEstadoAtual());
        telemetry.addData("Feeder Power", feeder.getPower());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Tempo Estado", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Busy", follower.isBusy());
    }
}