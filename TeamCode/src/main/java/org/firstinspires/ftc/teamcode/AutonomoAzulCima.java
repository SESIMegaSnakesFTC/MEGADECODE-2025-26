package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "AZUL CIMA")
public class AutonomoAzulCima extends OpMode {

    // MOTORES
    private DcMotor baseShooter = null;
    private DcMotorEx feeder;
    private Follower follower;

    // CONTROLADORES
    private Paths paths;
    private Timer pathTimer;
    private ShooterController shooterController;

    // CONSTANTES
    static final double TICKS_POR_REVOLUCAO = 560.0;
    static final double ROTACAO_ALVO_INIT = -1.135;
    static final double ROTACAO_ALVO_SHOOT2 = -0.56;
    static final double POWER_SETUP = 0.5;

    // TEMPORIZAÇÃO
    private static final double TEMPO_FEED_CIMA = 3.8;
    private static final double DELAY_APOS_FEED = 0.5;
    private static final double ESPERA_SHOOT = 7.796;

    // MÁQUINA DE ESTADOS
    public enum PathState {
        SETUP_SHOOTER,
        DESCERSHOOT1,
        ESPERA_SHOOT1,
        AJUSTEFEEDCIMA,
        FEEDCIMA_INICIAR,
        FEEDCIMA_EM_ANDAMENTO,
        FEEDCIMA_FINALIZAR,
        SETUP_SHOOTER2,
        VOLTASHOOT3,
        ESPERA_SHOOT3,
        POSICAOFINAL,
        DONE
    }

    private PathState pathState;

    // FUNÇÃO CRIADORA DE CAMINHOS
    public static class Paths {

        public PathChain DESCERSHOOT1;
        public PathChain AJUSTEFEEDCIMA;
        public PathChain FEEDCIMA;
        public PathChain VOLTASHOOT3;
        public PathChain POSICAOFINAL;

        public Paths(Follower follower) {
            DESCERSHOOT1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(20.700, 123.000), new Pose(54.200, 89.500))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            AJUSTEFEEDCIMA = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(54.200, 89.500), new Pose(51.000, 74.600))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            FEEDCIMA = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(51.000, 74.600), new Pose(15.000, 74.600))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            VOLTASHOOT3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(15.000, 74.600), new Pose(54.200, 89.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-159), Math.toRadians(-159))
                    .build();

            POSICAOFINAL = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(54.200, 89.500), new Pose(103.000, 33.400))
                    )
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }

    @Override
    public void init() {
        // CONFIGURAÇÃO BASE SHOOTER
        baseShooter = hardwareMap.get(DcMotor.class, "baseShooter");

        baseShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        baseShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        baseShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // CONFIGURAÇÃO FEEDER
        feeder = hardwareMap.get(DcMotorEx.class, "feeder");
        feeder.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        feeder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // INICIALIZA CLASSE LÓGICA SHOOTER
        shooterController = new ShooterController();
        shooterController.init(hardwareMap, feeder);

        // INICIALIZA PEDRO PATHING
        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower);

        pathTimer = new Timer();
        pathState = PathState.SETUP_SHOOTER;

        follower.setPose(
                new Pose(20.700,
                        123.000,
                        Math.toRadians(-45))
        );

        telemetry.addData("STATUS", "HARDWARE OK. SHOOTER AGUARDANDO.");
        telemetry.addData("POS INICIAL AZUL CIMA", follower.getPose().toString());
        telemetry.update();
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

            case SETUP_SHOOTER:
                if (baseShooter.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                    int posicaoAlvoTicks = (int) (ROTACAO_ALVO_INIT * TICKS_POR_REVOLUCAO);
                    baseShooter.setTargetPosition(posicaoAlvoTicks);
                    baseShooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    baseShooter.setPower(POWER_SETUP);
                }

                if (!baseShooter.isBusy()) {
                    baseShooter.setPower(0);
                    baseShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    setPathState(PathState.DESCERSHOOT1);
                }
                break;

            case DESCERSHOOT1:
                shooterController.iniciarCiclo3Bolinhas();

                follower.setMaxPower(1.0);
                follower.followPath(paths.DESCERSHOOT1, true);
                setPathState(PathState.ESPERA_SHOOT1);
                break;

            case ESPERA_SHOOT1:
                shooterController.update();

                if (pathTimer.getElapsedTimeSeconds() < 2.0) {
                    telemetry.addLine("Acelerando shooter...");
                }
                else if (shooterController.isReadyToShoot()) {
                    shooterController.comecarATirar();
                    telemetry.addLine("Atirando bolinhas...");
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
                }
                break;

            case FEEDCIMA_FINALIZAR:
                if (pathTimer.getElapsedTimeSeconds() >= DELAY_APOS_FEED) {
                    feeder.setPower(0);
                    setPathState(PathState.SETUP_SHOOTER2);
                }
                break;

            case SETUP_SHOOTER2:
                // MOVE A BASE SHOOTER
                if (baseShooter.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                    int posicaoAlvoTicks = (int) (ROTACAO_ALVO_SHOOT2 * TICKS_POR_REVOLUCAO);
                    baseShooter.setTargetPosition(posicaoAlvoTicks);
                    baseShooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    baseShooter.setPower(POWER_SETUP);
                }

                if (!baseShooter.isBusy()) {
                    baseShooter.setPower(0);
                    baseShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    setPathState(PathState.VOLTASHOOT3);
                }
                break;

            case VOLTASHOOT3:
                if (!follower.isBusy()) {
                    // LIGA SHOOTER
                    shooterController.iniciarCiclo3Bolinhas();

                    follower.setMaxPower(1);
                    follower.followPath(paths.VOLTASHOOT3, true);
                    setPathState(PathState.ESPERA_SHOOT3);
                }
                break;

            case ESPERA_SHOOT3:
                shooterController.update();

                if (pathTimer.getElapsedTimeSeconds() < 2.0) {
                }
                else if (shooterController.isReadyToShoot()) {
                    shooterController.comecarATirar();
                }
                else if (shooterController.isIdle()) {
                    setPathState(PathState.POSICAOFINAL);
                }
                else if (pathTimer.getElapsedTimeSeconds() >= ESPERA_SHOOT) {
                    shooterController.emergencyStop();
                    setPathState(PathState.POSICAOFINAL);
                }
                break;

            case POSICAOFINAL:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1.0);
                    follower.followPath(paths.POSICAOFINAL, true);
                    setPathState(PathState.DONE);
                }
                break;

            case DONE:
                // TUDO DESLIGADO
                feeder.setPower(0);
                shooterController.emergencyStop();
                break;
        }
    }

    @Override
    public void loop() {
        // ATUALIZA SEGUIDOR
        follower.update();

        // ATUALIZA MÁQUINA DE ESTADOS
        statePathUpdate();

        // MENSAGENS DE DEPURAÇÃO
        telemetry.addData("Estado", pathState);
        telemetry.addData("Estado Shoot", shooterController.getEstadoAtual());
        telemetry.addData("Bolinhas Atiradas", shooterController.getBolinhasAtiradas());
        telemetry.addData("Tempo Estado Shoot", shooterController.getTempoEstadoAtual());
        telemetry.addData("Feeder Power", feeder.getPower());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Tempo Estado", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Busy", follower.isBusy());

        // DEPURAÇÃO
        if (pathState == PathState.SETUP_SHOOTER) {
            telemetry.addData("Setup Shooter 1", "Movendo para %s Rots", ROTACAO_ALVO_INIT);
            telemetry.addData("Shooter Pos Atual", baseShooter.getCurrentPosition());
        } else if (pathState == PathState.SETUP_SHOOTER2) {
            telemetry.addData("Setup Shooter 2", "Movendo para %s Rots", ROTACAO_ALVO_SHOOT2);
            telemetry.addData("Shooter Pos Atual", baseShooter.getCurrentPosition());
        }
    }
}