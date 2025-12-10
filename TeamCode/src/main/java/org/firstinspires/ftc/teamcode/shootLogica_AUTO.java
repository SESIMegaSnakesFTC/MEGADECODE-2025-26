package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class shootLogica_AUTO {

    private Servo alavancaServo;
    private DcMotorSimple shooter;
    private ElapsedTime cronometroEstado = new ElapsedTime();
    private int bolinhasAtiradas = 0; // Contador de bolinhas já lançadas

    public enum EstadoShooter {
        INATIVO,
        PREPARANDO_RPM, // Liga o shooter e espera RPM
        AGUARDA_TRANSPORTE, // RPM Ok, esperando que o Autônomo ligue o feeder para transporte
        ATIRANDO_ALAVANCA, // Ativa a alavanca (para a última bolinha)
        RECARREGANDO // Tempo de espera para o shooter reestabilizar
    }
    private EstadoShooter estadoShooter;

    // CONSTANTES
    private double ALAVANCA_FECHADA = 0;
    private double ALAVANCA_ABERTA = 0.5;
    private double TEMPO_ALAVANCA_ATIVA = 0.3; // Tempo que a alavanca deve ficar aberta
    private double TEMPO_RECARGA = 0.3; // Tempo para o shooter reestabilizar entre disparos

    // VARIÁVEIS DE CONTROLE
    private int bolinhasParaAtirarTotal = 0;
    private double velocidadeShooter = 0;
    private double RPM_MINIMO_SHOOTER = 800;
    private double RPM_SHOOTER_ALVO = 1100;
    private double MAX_TEMPO_SHOOTER_PREPARANDO = 2;


    public void init(HardwareMap hardwareMap) {
        alavancaServo = hardwareMap.get(Servo.class, "alavancaServo");
        shooter = hardwareMap.get(DcMotorSimple.class, "shooter");
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);

        // Inicialização
        estadoShooter = EstadoShooter.INATIVO;
        alavancaServo.setPosition(ALAVANCA_FECHADA);
        shooter.setPower(0);
    }

    public void update() {
        switch (estadoShooter) {

            case INATIVO:
                // Aguardando comando para atirar.
                // Se atirarBolinhas for chamado, o próximo update vai para PREPARANDO_RPM
                break;

            case PREPARANDO_RPM:
                // Garante que o motor está ligado
                if (shooter.getPower() == 0) {
                    shooter.setPower(RPM_SHOOTER_ALVO / 1200.0); // Converte RPM Alvo para Potência (ex: 0.9)
                }

                // Espera o RPM atingir o mínimo ou timeout
                if (velocidadeShooter >= RPM_MINIMO_SHOOTER || cronometroEstado.seconds() > MAX_TEMPO_SHOOTER_PREPARANDO) {
                    cronometroEstado.reset();

                    if (bolinhasAtiradas < bolinhasParaAtirarTotal - 1) { // Ainda há bolinhas para o Feeder (1ª ou 2ª)
                        estadoShooter = EstadoShooter.AGUARDA_TRANSPORTE;
                    } else if (bolinhasAtiradas == bolinhasParaAtirarTotal - 1) { // Última bolinha (Alavanca)
                        estadoShooter = EstadoShooter.ATIRANDO_ALAVANCA;
                    } else {
                        // Todas as bolinhas já foram lançadas (deve ter ocorrido um erro no contador)
                        shooter.setPower(0);
                        estadoShooter = EstadoShooter.INATIVO;
                    }
                }
                break;

            case AGUARDA_TRANSPORTE:
                // O Autônomo deve ligar o motor de transporte (feeder) por um tempo fixo.
                // A Lógica de Lançamento espera o motor de transporte terminar (indicado pelo Autônomo).
                // Neste estado, o controle está com o Autônomo (via isReadyToFeed()).

                // Se o Autônomo já tiver ligado o transporte e voltado, a contagem é incrementada
                // pelo método `feederCycleComplete()`.

                // O update() não faz nada, apenas espera a chamada externa.
                break;

            case ATIRANDO_ALAVANCA:
                // Ativa a alavanca (Última bolinha)
                alavancaServo.setPosition(ALAVANCA_ABERTA);

                if (cronometroEstado.seconds() >= TEMPO_ALAVANCA_ATIVA) {
                    alavancaServo.setPosition(ALAVANCA_FECHADA);
                    bolinhasAtiradas++; // Última bolinha lançada
                    cronometroEstado.reset();
                    estadoShooter = EstadoShooter.RECARREGANDO;
                }
                break;

            case RECARREGANDO:
                // Espera o tempo de reestabilização
                if (cronometroEstado.seconds() >= TEMPO_RECARGA) {

                    if (bolinhasAtiradas < bolinhasParaAtirarTotal) {
                        // Ainda faltam bolinhas - Volta para rechecar RPM
                        cronometroEstado.reset();
                        estadoShooter = EstadoShooter.PREPARANDO_RPM;
                    } else {
                        // Todas as bolinhas foram atiradas
                        shooter.setPower(0);
                        estadoShooter = EstadoShooter.INATIVO;
                    }
                }
                break;
        }
    }

    // Metodo para definir quantas bolinhas atirar
    public void atirarBolinhas(int quantidade) {
        if (estadoShooter == EstadoShooter.INATIVO && quantidade > 0) {
            bolinhasParaAtirarTotal = quantidade;
            bolinhasAtiradas = 0; // Reseta o contador
            cronometroEstado.reset();
            estadoShooter = EstadoShooter.PREPARANDO_RPM;
        }
    }

    // Metodo para ser chamado pelo Autônomo após o tempo de transporte/subida.
    public void feederCycleComplete() {
        if (estadoShooter == EstadoShooter.AGUARDA_TRANSPORTE) {
            bolinhasAtiradas++;
            cronometroEstado.reset();
            estadoShooter = EstadoShooter.RECARREGANDO;
        }
    }

    // Metodo para o Autônomo saber se o RPM está pronto e ele deve ligar o Feeder (transporte)
    public boolean isReadyToFeed() {
        return estadoShooter == EstadoShooter.AGUARDA_TRANSPORTE;
    }

    // Metodo para o Autônomo saber se o Servo/Alavanca está ativo (para sincronização, se necessário)
    public boolean isFiringWithServo() {
        return estadoShooter == EstadoShooter.ATIRANDO_ALAVANCA;
    }

    // Retorna o estado atual
    public EstadoShooter getEstadoShooter() {
        return estadoShooter;
    }

    // Retorna quantas bolinhas faltam
    public int getBolinhasFaltantes() {
        return bolinhasParaAtirarTotal - bolinhasAtiradas;
    }

    // Metodo para atualizar velocidade do shooter (deve ser chamado periodicamente)
    public void atualizarVelocidadeShooter(double velocidadeAtual) {
        this.velocidadeShooter = velocidadeAtual;
    }

    // Metodo para verificar se o sistema está ocupado
    public boolean isBusy() {
        return estadoShooter != EstadoShooter.INATIVO;
    }

    // Metodo para parada de emergência
    public void emergencyStop() {
        estadoShooter = EstadoShooter.INATIVO;
        shooter.setPower(0);
        alavancaServo.setPosition(ALAVANCA_FECHADA);
        bolinhasParaAtirarTotal = 0;
        bolinhasAtiradas = 0;
    }
}