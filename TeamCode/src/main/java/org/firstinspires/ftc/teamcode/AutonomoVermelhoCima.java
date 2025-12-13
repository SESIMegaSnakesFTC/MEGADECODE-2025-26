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

@Autonomous(name = "VERMELHO CIMA")
public class AutonomoVermelhoCima extends OpMode {
    // Hardware
    private DcMotor baseShooter = null;
    private DcMotorEx feeder;
    private Follower follower;

    // Controladores
    private Paths paths;
    private Timer pathTimer;
    private ShooterController shooterController;

    // Constantes do Motor
    static final double TICKS_POR_REVOLUCAO = 560.0;
    static final double ROTACAO_ALVO_INIT = 1.135;
    static final double ROTACAO_ALVO_SHOOT2 = 0.56;
    static final double POWER_SETUP = 0.5;

    // Constantes de tempo
    private static final double TEMPO_FEED_CIMA = 3.8;
    private static final double DELAY_APOS_FEED = 0.5;
    private static final double ESPERA_SHOOT = 7.796;

    // Enum PathState ATUALIZADA
    public enum PathState {
        SETUP_SHOOTER,          // Setup inicial (ROTACAO_ALVO_INIT)
        DESCERSHOOT1,
        ESPERA_SHOOT1,           // Primeiro Tiro
        AJUSTEFEEDCIMA,
        FEEDCIMA_INICIAR,
        FEEDCIMA_EM_ANDAMENTO,
        FEEDCIMA_FINALIZAR,
        SETUP_SHOOTER2,         // Ajusta para ROTACAO_ALVO_SHOOT2
        VOLTASHOOT3,
        ESPERA_SHOOT3,           // Segundo Tiro
        POSICAOFINAL,           // NOVO: Movimento para a posição final/estacionamento
        DONE
    }

    private PathState pathState;

    public static class Paths {
        public PathChain DESCERSHOOT1;
        public PathChain AJUSTEFEEDCIMA;
        public PathChain FEEDCIMA;
        public PathChain VOLTASHOOT3;
        public PathChain POSICAOFINAL; // NOVO CAMINHO

        public Paths(Follower follower) {
            DESCERSHOOT1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(123.400, 123.000), new Pose(90.000, 89.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-134), Math.toRadians(-140))
                    .build();

            AJUSTEFEEDCIMA = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(95.700, 95.300), new Pose(93.000, 74.600))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-134), Math.toRadians(0))
                    .build();

            // Aumentado o X final no passo anterior, usando 128.000
            FEEDCIMA = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(93.000, 74.600), new Pose(128.000, 74.600))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(7))
                    .build();

            VOLTASHOOT3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(128.000, 74.600), new Pose(104.300, 102.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            // NOVO CAMINHO: POSICAOFINAL
            // Nota: O ponto inicial (102.000, 105.600) não corresponde ao final de VOLTASHOOT3 (104.300, 103.300),
            // mas estou usando o que foi fornecido na sua solicitação.
            POSICAOFINAL = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(104.300, 102.000),
                            new Pose(52.600, 20.400)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(57),
                            Math.toRadians(90))
                    .build();
        }
    }

    @Override
    public void init() {
        // ** Mapeamento e Configuração do baseShooter **
        baseShooter = hardwareMap.get(DcMotor.class, "baseShooter");

        baseShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        baseShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        baseShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Inicializa feeder
        feeder = hardwareMap.get(DcMotorEx.class, "feeder");
        feeder.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        feeder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Inicializa shooter controller
        shooterController = new ShooterController();
        shooterController.init(hardwareMap, feeder);

        // Inicializa follower e paths
        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower);

        pathTimer = new Timer();
        pathState = PathState.SETUP_SHOOTER;

        follower.setPose(
                new Pose(123.400,
                        123.000,
                        Math.toRadians(-134))
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
                // AGORA SÓ AVANÇA APÓS O TEMPO DE FEED EXPIRAR
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
                // Move para ROTACAO_ALVO_SHOOT2
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
                    // Liga shooter para último ciclo (após ajuste da base)
                    shooterController.iniciarCiclo3Bolinhas();

                    follower.setMaxPower(1);
                    follower.followPath(paths.VOLTASHOOT3, true);
                    setPathState(PathState.ESPERA_SHOOT3);
                }
                break;

            case ESPERA_SHOOT3:
                // Lógica do segundo tiro
                shooterController.update();

                if (pathTimer.getElapsedTimeSeconds() < 2.0) {
                    // Aguarda shooter acelerar
                }
                else if (shooterController.isReadyToShoot()) {
                    shooterController.comecarATirar();
                }
                else if (shooterController.isIdle()) {
                    setPathState(PathState.POSICAOFINAL); // Transiciona para o novo caminho final
                }
                else if (pathTimer.getElapsedTimeSeconds() >= ESPERA_SHOOT) {
                    shooterController.emergencyStop();
                    setPathState(PathState.POSICAOFINAL); // Transiciona para o novo caminho final
                }
                break;

            case POSICAOFINAL: // NOVO ESTADO
                if (!follower.isBusy()) {
                    follower.setMaxPower(1.0);
                    follower.followPath(paths.POSICAOFINAL, true);
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

        // Telemetria do Shooter (ADICIONADA)
        if (pathState == PathState.SETUP_SHOOTER) {
            telemetry.addData("Setup Shooter 1", "Movendo para %s Rots", ROTACAO_ALVO_INIT);
            telemetry.addData("Shooter Pos Atual", baseShooter.getCurrentPosition());
        } else if (pathState == PathState.SETUP_SHOOTER2) {
            telemetry.addData("Setup Shooter 2", "Movendo para %s Rots", ROTACAO_ALVO_SHOOT2);
            telemetry.addData("Shooter Pos Atual", baseShooter.getCurrentPosition());
        }
    }
}