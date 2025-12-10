package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
// IMPORTAÇÃO DA NOVA LÓGICA
import org.firstinspires.ftc.teamcode.shootLogica_AUTO;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "VERMELHO CIMA")
public class AutonomoVermelhoCima extends OpMode {

    private DcMotorEx feeder; // ESTE É O MOTOR ÚNICO: Coleta E Transporte
    private Follower follower;
    private Paths paths;
    private Timer pathTimer;

    // INSTANCIA DA LÓGICA DO LANÇADOR
    private shootLogica_AUTO lancador = new shootLogica_AUTO();

    // VARIÁVEIS DE CONTROL
    private ElapsedTime cicloFeederTimer = new ElapsedTime();
    private final double TEMPO_TRANSPORTE = 0.4; // TEMPO FEEDER EMPURRANDO BOLINHA
    private final double VELOCIDADE_COLETA = -1.0;
    private final double VELOCIDADE_TRANSPORTE = -1.0;

    public enum PathState {

        // ESTADOS DE PREPARAR
        INICIAR_PREPARO,
        SHOOT_WAIT_RPM,
        SHOOT_WAIT_TRANSPORTE_DONE,

        // ESTADOS DOS CAMINHOS
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
            // ... (Seus BezierLines inalterados) ...
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
        lancador.init(hardwareMap); // INICIALIZA A LÓGICA DO LANÇADOR

        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower);

        pathTimer = new Timer();
        pathState = PathState.INICIAR_PREPARO; // COMEÇA PREPARANDO

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

    // NOVO METODO: Função simulada para calcular RPM (você deve implementar a leitura de encoder real)
    private double calcularRpmShooter() {
        // Se o motor for DcMotorEx, use encoder:
        // double encoderTicksPerRevolution = 28; // Exemplo para motor Yellow Jacket 5202
        // return (feeder.getVelocity() * 60) / encoderTicksPerRevolution;

        // Simulação: Se o lançador estiver ocupado, assume que o RPM está perto do alvo
        if (lancador.isBusy()) {
            return 1100.0;
        }
        return 0.0;
    }

    private void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        feeder.setPower(0); // Garante que o feeder para ao mudar de estado
    }

    private void statePathUpdate() {

        // Se a flywheel estiver na fase ATIRANDO_ALAVANCA, o Autônomo não deve interferir no motor feeder
        if (lancador.getEstadoShooter() == shootLogica_AUTO.EstadoShooter.ATIRANDO_ALAVANCA) {
            // O update() do lancador cuidará do servo e do fim do ciclo
            return;
        }

        switch (pathState) {

            case INICIAR_PREPARO:
                // Liga lançamento de 3 bolinhas
                lancador.atirarBolinhas(3);

                // O processo de PREPARANDO_RPM começará no loop()
                setPathState(PathState.DESCER_SHOOT1);
                break;

            case DESCER_SHOOT1:
                // FLYWHEEL LIGADA E CARREGANDO DURANTE O CAMINHO
                follower.setMaxPower(1);
                follower.followPath(paths.DESCERSHOOT1, true);

                setPathState(PathState.AJUSTE_FEED_MEIO);
                break;

            case SHOOT_WAIT_RPM:
                // ESTADO DE SINCRONIZAÇÃO E LANÇAMENTO

                // 1. O Autônomo espera o RPM ficar pronto
                if (lancador.isReadyToFeed()) {
                    // 2. RPM OK. Liga o motor UNICO (feeder) para o TRANSPORTE/SUBIDA
                    feeder.setPower(VELOCIDADE_TRANSPORTE);
                    cicloFeederTimer.reset();
                    setPathState(PathState.SHOOT_WAIT_TRANSPORTE_DONE);
                } else if (!lancador.isBusy()) {
                    // Todas as bolinhas atiradas (fim do ciclo)
                    setPathState(PathState.DONE);
                }
                break;

            case SHOOT_WAIT_TRANSPORTE_DONE:
                // 1. Espera o tempo necessário para empurrar a bolinha (transporte)
                if (cicloFeederTimer.seconds() >= TEMPO_TRANSPORTE) {
                    feeder.setPower(0); // DESLIGA O MOTOR DE TRANSPORTE
                    lancador.feederCycleComplete(); // Informa a lógica que o transporte terminou (incrementa bolinhasAtiradas e vai para RECARREGANDO)

                    // 2. Decide a próxima ação
                    if (lancador.getBolinhasFaltantes() == 1) {
                        // Falta apenas a última bolinha (servo/alavanca), então vai para navegação para coletar as últimas
                        setPathState(PathState.AJUSTE_FEED_MEIO);
                    } else if (lancador.getBolinhasFaltantes() > 1) {
                        // Falta mais de uma bolinha (primeiro ou segundo tiro), então volta para rechecar RPM
                        setPathState(PathState.SHOOT_WAIT_RPM);
                    } else {
                        // Não deve acontecer, mas por segurança
                        setPathState(PathState.DONE);
                    }
                }
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
                    feeder.setPower(VELOCIDADE_COLETA); // LIGA O FEEDER (COLETA)
                    follower.setMaxPower(0.25);
                    follower.followPath(paths.FEEDMEIO, true);
                    setPathState(PathState.AJUSTE_VOLTA);
                }
                break;

            case AJUSTE_VOLTA:
                if (!follower.isBusy()) {
                    feeder.setPower(0); // DESLIGA O FEEDER (COLETA)
                    follower.setMaxPower(0.3);
                    follower.followPath(paths.AJUSTEVOLTA, true);
                    setPathState(PathState.VOLTA_SHOOT2);
                }
                break;

            case VOLTA_SHOOT2:
                if (!follower.isBusy()) {
                    // FLWHEEL JÁ FOI LIGADA EM INICIAR_PREPARO, MAS GARANTIMOS QUE ELA ESTÁ EM PROCESSO DE CARGA
                    if (!lancador.isBusy()) { lancador.atirarBolinhas(lancador.getBolinhasFaltantes()); }

                    follower.setMaxPower(1);
                    follower.followPath(paths.VOLTASHOOT2, true);
                    setPathState(PathState.SHOOT_WAIT_RPM); // Vai para o estado de tiro (2º tiro da primeira sequência, se aplicável)
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
                    feeder.setPower(VELOCIDADE_COLETA); // LIGA O FEEDER (COLETA)
                    follower.setMaxPower(0.3);
                    follower.followPath(paths.FEEDCIMA, true);
                    setPathState(PathState.VOLTA_SHOOT3);
                }
                break;

            case VOLTA_SHOOT3:
                if (!follower.isBusy()) {
                    feeder.setPower(0); // DESLIGA O FEEDER (COLETA)
                    // FLWHEEL JÁ FOI LIGADA EM INICIAR_PREPARO, MAS GARANTIMOS QUE ELA ESTÁ EM PROCESSO DE CARGA
                    if (!lancador.isBusy()) { lancador.atirarBolinhas(lancador.getBolinhasFaltantes()); }

                    follower.setMaxPower(1);
                    follower.followPath(paths.VOLTASHOOT3, true);
                    setPathState(PathState.SHOOT_WAIT_RPM); // Vai para o estado de tiro (último tiro)
                }
                break;

            case DONE:
                // Garante que tudo para no final
                lancador.emergencyStop();
                feeder.setPower(0);
                break;
        }
    }


    @Override
    public void loop() {

        lancador.atualizarVelocidadeShooter(calcularRpmShooter());
        lancador.update();

        follower.update();
        statePathUpdate();

        telemetry.addData("Estado Autônomo", pathState);
        telemetry.addData("Estado Flywheel", lancador.getEstadoShooter());
        telemetry.addData("Bolinhas Faltantes", lancador.getBolinhasFaltantes());
        telemetry.addData("Feeder Power", feeder.getPower());
        telemetry.addData("RPM Atual", calcularRpmShooter());

        telemetry.update();
    }
}