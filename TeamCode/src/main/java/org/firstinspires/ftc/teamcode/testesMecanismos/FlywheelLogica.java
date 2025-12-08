package org.firstinspires.ftc.teamcode.testesMecanismos;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class FlywheelLogica {

    private Servo alavancaServo;
    private DcMotorSimple shooter;
    private ElapsedTime cronometroEstado = new ElapsedTime();

    private enum EstadoShooter {
        INATIVO,
        GIRANDO,
        ATIRANDO,
        RESET
    }
    private EstadoShooter estadoShooter;

    // CONSTANTES ALAVANCA
    private double ALAVANCA_FECHADA = 0;
    private double ALAVANCA_ABERTA = 0.5;
    private double TEMPO_ALAVANCA_ATIVA = 0.3;

    // CONSTANTES SHOOTER
    private int bolinhasParaAtirar = 0;
    private boolean usarAlavanca = false;
    private double velocidadeShooter = 0;
    private double RPM_MINIMO_SHOOTER = 800;
    private double RPM_SHOOTER_ALVO = 1100;
    private double MAX_TEMPO_SHOOTER_GIRANDO = 2;
    private double TEMPO_ENTRE_DISPAROS = 0.4;

    public void init(HardwareMap hardwareMap) {
        alavancaServo = hardwareMap.get(Servo.class, "alavancaServo");
        shooter = hardwareMap.get(DcMotorSimple.class, "shooter");
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);

        //Inicialização
        estadoShooter = EstadoShooter.INATIVO;

        //Começa com alavanca fechada
        alavancaServo.setPosition(ALAVANCA_FECHADA);
        shooter.setPower(0);
    }

    public void update() {
        switch (estadoShooter) {
            case INATIVO:
                // Estado inicial - sistema parado
                if (bolinhasParaAtirar > 0) {
                    // Liga o shooter
                    shooter.setPower(RPM_SHOOTER_ALVO);
                    cronometroEstado.reset();
                    estadoShooter = EstadoShooter.GIRANDO;

                    // Se for a última bolinha (1 restante), prepara para usar alavanca
                    usarAlavanca = (bolinhasParaAtirar == 1);
                }
                break;

            case GIRANDO:
                // Espera o shooter atingir velocidade mínima
                if (velocidadeShooter > RPM_MINIMO_SHOOTER || cronometroEstado.seconds() > MAX_TEMPO_SHOOTER_GIRANDO) {

                    // Se for usar alavanca, ativa ela antes de atirar
                    if (usarAlavanca) {
                        alavancaServo.setPosition(ALAVANCA_ABERTA);
                    }

                    cronometroEstado.reset();
                    estadoShooter = EstadoShooter.ATIRANDO;
                }
                break;

            case ATIRANDO:
                // Tempo para a bolinha ser lançada
                if (cronometroEstado.seconds() > TEMPO_ENTRE_DISPAROS) {
                    // Decrementa contador de bolinhas
                    bolinhasParaAtirar = bolinhasParaAtirar - 1;

                    // Se usou alavanca, fecha ela
                    if (usarAlavanca) {
                        alavancaServo.setPosition(ALAVANCA_FECHADA);
                    }

                    cronometroEstado.reset();
                    estadoShooter = EstadoShooter.RESET;
                }
                break;

            case RESET:
                // Tempo de reset entre disparos
                if (cronometroEstado.seconds() > TEMPO_ENTRE_DISPAROS) {
                    if (bolinhasParaAtirar > 0) {
                        // Próxima bolinha - verifica se precisa de alavanca
                        usarAlavanca = (bolinhasParaAtirar == 1);
                        estadoShooter = EstadoShooter.GIRANDO;
                    } else {
                        // Todas as bolinhas foram atiradas
                        shooter.setPower(0);
                        estadoShooter = EstadoShooter.INATIVO;
                        usarAlavanca = false;
                    }
                }
                break;
        }
    }

    // Metodo para definir quantas bolinhas atirar
    public void atirarBolinhas(int quantidade) {
        if (estadoShooter == EstadoShooter.INATIVO && quantidade > 0) {
            bolinhasParaAtirar = quantidade;
        }
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
        bolinhasParaAtirar = 0;
        usarAlavanca = false;
    }
}