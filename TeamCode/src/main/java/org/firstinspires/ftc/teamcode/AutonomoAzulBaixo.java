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

@Autonomous(name = "AZUL BAIXO")
public class AutonomoAzulBaixo extends OpMode {

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
    static final double ROTACAO_ALVO_INIT = -0.095; // POSIÇÃO SHOOT 1
    static final double ROTACAO_ALVO_SHOOT2 = -0.99; // POSIÇÃO SHOOT 2
    static final double POWER_SETUP = 0.5; // FORÇA DE GIRO BASE

    // CONSTANTE TEMPO FEED/SHOOT
    private static final double TEMPO_FEED_BAIXO = 3.98;
    private static final double DELAY_APOS_FEED = 0.5;
    private static final double ESPERA_SHOOT = 8.0;

    public enum PathState {

        // AJUSTE BASE
        SETUP_SHOOTER,
        IDAFRENTEINICIAL,  // LIGA O SHOOTER
        CAMINHOSHOOT1,
        ESPERA_SHOOT1,

        IDAFEEDBAIXO,
        FEEDBAIXO_INICIAR,
        FEEDBAIXO_EM_ANDAMENTO,
        FEEDBAIXO_FINALIZAR,
        BATEMEIO,

        SETUP_SHOOTER2,
        CAMINHOSHOOT2,
        ESPERA_SHOOT2,
        POSICAOFINAL,
        DONE
    }

    private PathState pathState;

    // FUNÇÃO CRIADORA DE CAMINHOS
    public static class Paths {

        public PathChain IDAFRENTEINICIAL;
        public PathChain CAMINHOSHOOT1;
        public PathChain IDAFEEDBAIXO;
        public PathChain FEEDBAIXO;
        public PathChain BATEMEIO;
        public PathChain CAMINHOSHOOT2;
        public PathChain POSICAOFINAL;

        public Paths(Follower follower) {
            IDAFRENTEINICIAL = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(88.000, 8.000),
                            new Pose(88.000, 30.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            CAMINHOSHOOT1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(88.000, 30.000), new Pose(50.000, 94.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            IDAFEEDBAIXO = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(50.000, 94.000), new Pose(92.700, 41.500))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            FEEDBAIXO = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(92.700, 41.500), new Pose(135.300, 41.500))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            BATEMEIO = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(135.300, 41.500), new Pose(116.700, 57.895))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            CAMINHOSHOOT2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(116.700, 57.895), new Pose(42.100, 105.600))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-33), Math.toRadians(-33))
                    .build();

            POSICAOFINAL = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(42.100, 105.600), new Pose(103.000, 33.400))
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

        // ININCIALIZA CLASSE LÓGICA SHOOTER
        shooterController = new ShooterController();
        shooterController.init(hardwareMap, feeder);

        // INICIALIZA PEDRO PATHING
        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower);

        pathTimer = new Timer();
        pathState = PathState.SETUP_SHOOTER;

        follower.setPose(
                new Pose(88.000,
                        8.000,
                        Math.toRadians(90))
        );

        telemetry.addData("STATUS", "HARDWARE OK. SHOOTER AGUARDANDO.");
        telemetry.addData("POS INICIAL AJUSTADA", follower.getPose().toString());
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

    // MÁQUINA DE ESTADOS
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
                    setPathState(PathState.IDAFRENTEINICIAL);
                }
                break;

            case IDAFRENTEINICIAL:
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
                if (pathTimer.getElapsedTimeSeconds() < 2.0) {
                    telemetry.addLine("Acelerando shooter...");
                }
                else if (shooterController.isReadyToShoot()) {
                    shooterController.comecarATirar();
                    telemetry.addLine("Atirando bolinhas...");
                }
                else if (shooterController.isIdle()) {
                    setPathState(PathState.IDAFEEDBAIXO);
                }
                else if (pathTimer.getElapsedTimeSeconds() >= ESPERA_SHOOT) {
                    shooterController.emergencyStop();
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

            case FEEDBAIXO_INICIAR: // NOVO ESTADO
                if (!follower.isBusy()) {
                    feeder.setPower(-1);
                    follower.setMaxPower(0.30);
                    follower.followPath(paths.FEEDBAIXO, true);
                    setPathState(PathState.FEEDBAIXO_EM_ANDAMENTO);
                }
                break;

            case FEEDBAIXO_EM_ANDAMENTO:
                if (pathTimer.getElapsedTimeSeconds() >= TEMPO_FEED_BAIXO || !follower.isBusy()) {
                    if (follower.isBusy()) {
                        follower.breakFollowing();
                    }
                    setPathState(PathState.FEEDBAIXO_FINALIZAR);
                }
                break;

            case FEEDBAIXO_FINALIZAR:
                if (pathTimer.getElapsedTimeSeconds() >= DELAY_APOS_FEED) {
                    feeder.setPower(0);
                    setPathState(PathState.BATEMEIO);
                }
                break;

            case BATEMEIO:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.7);
                    follower.followPath(paths.BATEMEIO, true);
                    setPathState(PathState.SETUP_SHOOTER2);
                }
                break;

            case SETUP_SHOOTER2:
                // MOVE A BASE DO SHOOTER
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
                    shooterController.iniciarCiclo3Bolinhas();

                    follower.setMaxPower(1.0);
                    follower.followPath(paths.CAMINHOSHOOT2, true);
                    setPathState(PathState.ESPERA_SHOOT2);
                }
                break;

            case ESPERA_SHOOT2:
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
        // ATUALIZA O SEGUIDOR
        follower.update();

        // CHAMA E ATUALIZA CLASSE DE CONTROLE INDIVIDUAL DO SHOOTER
        shooterController.update();

        // ATUALIZA MÁQUINA DE ESTADO
        statePathUpdate();

        // MENSAGENS DE DEPURAÇÃO
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
