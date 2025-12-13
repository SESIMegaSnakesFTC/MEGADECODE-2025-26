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

@Autonomous(name = "VERMELHO BAIXO")
public class AutonomoVermelhoBaixo extends OpMode {

    // MOTORES/XERIFE
    private DcMotor baseShooter = null;
    private DcMotorEx feeder;
    private Follower follower;

    // CONTROLES
    private Paths paths;
    private Timer pathTimer;

    // IMPORTAÇÃO DE OUTRA CLASSE
    private ShooterController shooterController;

    // CONSTANTES BASE SHOOTER
    static final double TICKS_POR_REVOLUCAO = 560.0;
    static final double ROTACAO_ALVO_INIT = 0.095; // POSIÇÃO SHOOT 1
    static final double ROTACAO_ALVO_SHOOT2 = 0.99; // POSIÇÃO SHOOT 2
    static final double POWER_SETUP = 0.5; // FORÇA DE GIRO BASE

    // CONSTANTE TEMPO FEED/SHOOT
    private static final double TEMPO_FEED_MEIO = 2.0;
    private static final double DELAY_APOS_FEED = 0.5;
    private static final double ESPERA_SHOOT = 8.0;

    public enum PathState {

        // AJUSTE BASE
        SETUP_SHOOTER,
        IDAFRENTEINICIAL,       // LIGA O SHOOTER
        CAMINHOSHOOT1,
        ESPERA_SHOOT1,
        IDAFEEDMEIO,
        FEEDMEIO_INICIAR,
        FEEDMEIO_EM_ANDAMENTO,
        FEEDMEIO_FINALIZAR,
        AJUSTEVOLTA,
        SETUP_SHOOTER2,
        CAMINHOSHOOT2,
        ESPERA_SHOOT2,
        POSICAOFINAL,
        DONE
    }

    private PathState pathState;

    public static class Paths {
        public PathChain IDAFRENTEINICIAL;
        public PathChain CAMINHOSHOOT1;
        public PathChain IDAFEEDMEIO;
        public PathChain FEEDMEIO;
        public PathChain AJUSTEVOLTA;
        public PathChain CAMINHOSHOOT2;
        public PathChain POSICAOFINAL;

        public Paths(Follower follower) {
            IDAFRENTEINICIAL = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(56.000, 8.000),
                            new Pose(56.000, 30.000)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            CAMINHOSHOOT1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(56.000, 30.000),
                            new Pose(98.000, 98.000)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            IDAFEEDMEIO = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(102.000, 101.000),
                            new Pose(45.000, 65.000)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(57),
                            Math.toRadians(180))
                    .build();

            FEEDMEIO = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(45.000, 65.000),
                            new Pose(12.000, 65.000)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            AJUSTEVOLTA = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(16.000, 65.000),
                            new Pose(27.000, 65.000)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(180))
                    .build();

            CAMINHOSHOOT2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(27.000, 65.000),
                            new Pose(102.000, 105.600)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(-149),
                            Math.toRadians(-149))
                    .build();

            POSICAOFINAL = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(102.000, 105.600),
                            new Pose(38.600, 33.400)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(57),
                            Math.toRadians(90))
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

        // ININCIALIZA CLASSE LÓGICA SHOOTER
        shooterController = new ShooterController();
        shooterController.init(hardwareMap, feeder);

        // INICIALIZA PEDRO PATHING
        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower);

        pathTimer = new Timer();
        pathState = PathState.SETUP_SHOOTER;

        follower.setPose(
                new Pose(56.000,
                        8.000,
                        Math.toRadians(90))
        );

        telemetry.addData("STATUS", "HARDWARE OK. SHOOTER AGUARDANDO.");
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

                // BASE SHOOT EM 0.095
                if (baseShooter.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                    int posicaoAlvoTicks = (int) (ROTACAO_ALVO_INIT * TICKS_POR_REVOLUCAO);
                    baseShooter.setTargetPosition(posicaoAlvoTicks);
                    baseShooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    baseShooter.setPower(POWER_SETUP);
                }

                if (!baseShooter.isBusy()) {
                    baseShooter.setPower(0);
                    baseShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    setPathState(PathState.IDAFRENTEINICIAL);
                }
                break;

            case IDAFRENTEINICIAL:

                // LIGA O SHOOTER ENQUANTO SE MOVE
                shooterController.iniciarCiclo3Bolinhas();

                follower.setMaxPower(1.0);
                follower.followPath(paths.IDAFRENTEINICIAL, true);
                setPathState(PathState.CAMINHOSHOOT1);
                break;

            case CAMINHOSHOOT1:

                if (!follower.isBusy()) {
                    follower.setMaxPower(1.0);
                    follower.followPath(paths.CAMINHOSHOOT1, true);
                    setPathState(PathState.ESPERA_SHOOT1);
                }
                break;

            case ESPERA_SHOOT1:
                // shooterController.update() já é chamado no loop()

                // FASE 1: Aguarda shooter acelerar
                if (pathTimer.getElapsedTimeSeconds() < 2.0) {
                    telemetry.addLine("Acelerando shooter...");
                }
                // FASE 2: Começa a atirar quando shooter estiver pronto
                // O feeder só liga AQUI dentro do método 'comecarATirar()'
                else if (shooterController.isReadyToShoot()) {
                    shooterController.comecarATirar();
                    telemetry.addLine("Atirando bolinhas...");
                }
                // FASE 3: Quando ciclo terminar, vai para próximo estado
                else if (shooterController.isIdle()) {
                    setPathState(PathState.IDAFEEDMEIO);
                }
                // FASE 4: Timeout de segurança
                else if (pathTimer.getElapsedTimeSeconds() >= ESPERA_SHOOT) {
                    shooterController.emergencyStop();
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
                    feeder.setPower(-1);

                    // ** CORREÇÃO DE VELOCIDADE: Aumentando para garantir o movimento **
                    // Aumentando de 0.15 para 0.30 (valor de stiction mais seguro)
                    follower.setMaxPower(0.30);
                    follower.followPath(paths.FEEDMEIO, true);

                    setPathState(PathState.FEEDMEIO_EM_ANDAMENTO);
                }
                break;

            case FEEDMEIO_EM_ANDAMENTO:
                if (pathTimer.getElapsedTimeSeconds() >= TEMPO_FEED_MEIO || !follower.isBusy()) {
                    if (follower.isBusy()) {
                        follower.breakFollowing();
                    }
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
                    follower.setMaxPower(0.7);
                    follower.followPath(paths.AJUSTEVOLTA, true);
                    setPathState(PathState.SETUP_SHOOTER2);
                }
                break;

            case SETUP_SHOOTER2:
                // Move para ROTACAO_ALVO_SHOOT2 (0.99)
                if (baseShooter.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                    int posicaoAlvoTicks = (int) (ROTACAO_ALVO_SHOOT2 * TICKS_POR_REVOLUCAO);
                    baseShooter.setTargetPosition(posicaoAlvoTicks);
                    baseShooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    baseShooter.setPower(POWER_SETUP);
                }

                if (!baseShooter.isBusy()) {
                    baseShooter.setPower(0);
                    baseShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    setPathState(PathState.CAMINHOSHOOT2);
                }
                break;

            case CAMINHOSHOOT2:
                if (!follower.isBusy()) {
                    // Liga shooter para próximo ciclo
                    shooterController.iniciarCiclo3Bolinhas();

                    follower.setMaxPower(1.0);
                    follower.followPath(paths.CAMINHOSHOOT2, true);
                    setPathState(PathState.ESPERA_SHOOT2);
                }
                break;

            case ESPERA_SHOOT2:
                // shooterController.update() já é chamado no loop()

                if (pathTimer.getElapsedTimeSeconds() < 2.0) {
                    telemetry.addLine("Acelerando shooter...");
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
                feeder.setPower(0);
                shooterController.emergencyStop();
                break;
        }
    }

    @Override
    public void loop() {
        // Atualiza follower (movimento)
        follower.update();

        // ** CHAMADA CRÍTICA: Atualiza o controlador de shoot em CADA LOOP **
        shooterController.update();

        // Atualiza máquina de estados
        statePathUpdate();

        // Telemetria detalhada
        telemetry.addData("Estado", pathState);
        telemetry.addData("Estado Shoot", shooterController.getEstadoAtual());

        if (pathState == PathState.SETUP_SHOOTER) {
            telemetry.addData("Setup Shooter 1", "Movendo para %s Rots", ROTACAO_ALVO_INIT);
            telemetry.addData("Shooter Pos Atual", baseShooter.getCurrentPosition());
        } else if (pathState == PathState.SETUP_SHOOTER2) {
            telemetry.addData("Setup Shooter 2", "Movendo para %s Rots", ROTACAO_ALVO_SHOOT2);
            telemetry.addData("Shooter Pos Atual", baseShooter.getCurrentPosition());
        }

        telemetry.addData("Feeder Power", feeder.getPower());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Tempo Estado", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Busy", follower.isBusy());
    }
}