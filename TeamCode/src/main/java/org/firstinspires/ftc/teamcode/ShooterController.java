package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ShooterController {
    // Hardware
    private DcMotorEx shooter;
    private DcMotorEx feeder;
    private Servo alavancaServo;

    // Timers
    private ElapsedTime stateTimer = new ElapsedTime();
    private ElapsedTime shooterTimer = new ElapsedTime(); // Timer separado para shooter

    // Estados do ciclo de shoot
    public enum ShootState {
        IDLE,
        ACELERANDO_SHOOTER,
        PRONTO_PARA_ATIRAR,
        ATIRANDO_BOLINHA_1,
        ESPERANDO_1_2,
        ATIRANDO_BOLINHA_2,
        ESPERANDO_2_3,
        PREPARANDO_BOLINHA_3,
        ATIRANDO_BOLINHA_3,
        FINALIZANDO
    }

    private ShootState currentState = ShootState.IDLE;

    // Constantes - AJUSTADAS
    private static final double TEMPO_ACELERACAO = 2.0;
    private static final double TEMPO_FEEDER_BOLINHA = 0.8;  // Tempo de alimentação por bolinha
    private static final double TEMPO_ENTRE_BOLINHAS = 1.0;  // REDUZIDO: tempo entre bolinhas
    private static final double TEMPO_SERVO_ACIONA = 0.3;    // Tempo servo acionado
    private static final double TEMPO_PARADO_ATIRAR = 0.3;   // Tempo estabilização antes de atirar

    // Tempo TOTAL que o shooter deve ficar ligado
    private static final double TEMPO_TOTAL_SHOOTER_LIGADO = 7.0; // Tempo suficiente para 3 bolinhas

    // Constantes de hardware
    private static final double POTENCIA_SHOOTER = 1.0;
    private static final double POTENCIA_FEEDER = -1.0;
    private static final double POSICAO_SERVO_ATIRAR = 0.8;
    private static final double POSICAO_SERVO_REPOUSO = 0.2;

    // Controle
    private int bolinhasAtiradas = 0;
    private boolean cicloEmAndamento = false;
    private boolean shooterLigado = false;
    private boolean feederLigado = false;

    public void init(HardwareMap hardwareMap, DcMotorEx feederMotor) {
        try {
            shooter = hardwareMap.get(DcMotorEx.class, "shooter");
            shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

            this.feeder = feederMotor;

            alavancaServo = hardwareMap.get(Servo.class, "alavancaServo");
            alavancaServo.setPosition(POSICAO_SERVO_REPOUSO);

        } catch (Exception e) {
            System.out.println("Erro ao inicializar ShooterController: " + e.getMessage());
            throw e;
        }
    }

    /**
     * Liga o shooter e começa contagem de tempo
     */
    private void ligarShooter() {
        if (shooter != null && !shooterLigado) {
            shooter.setPower(POTENCIA_SHOOTER);
            shooterLigado = true;
            shooterTimer.reset(); // Começa a contar tempo do shooter ligado
        }
    }

    /**
     * Desliga o shooter
     */
    private void desligarShooter() {
        if (shooter != null && shooterLigado) {
            shooter.setPower(0);
            shooterLigado = false;
        }
    }

    /**
     * Liga o feeder
     */
    private void ligarFeeder() {
        if (feeder != null && !feederLigado) {
            feeder.setPower(POTENCIA_FEEDER);
            feederLigado = true;
        }
    }

    /**
     * Desliga o feeder
     */
    private void desligarFeeder() {
        if (feeder != null && feederLigado) {
            feeder.setPower(0);
            feederLigado = false;
        }
    }

    /**
     * Inicia o ciclo completo de 3 bolinhas
     */
    public void iniciarCiclo3Bolinhas() {
        if (currentState == ShootState.IDLE) {
            currentState = ShootState.ACELERANDO_SHOOTER;
            ligarShooter(); // Liga shooter imediatamente
            bolinhasAtiradas = 0;
            cicloEmAndamento = true;
            stateTimer.reset();
        }
    }

    public void update() {
        if (stateTimer == null) return;

        // VERIFICAÇÃO DE SEGURANÇA: se shooter está ligado há muito tempo, desliga
        if (shooterLigado && shooterTimer.seconds() >= TEMPO_TOTAL_SHOOTER_LIGADO) {
            desligarShooter();
        }

        switch (currentState) {
            case ACELERANDO_SHOOTER:
                // Shooter já está ligado, aguarda acelerar
                if (stateTimer.seconds() >= TEMPO_ACELERACAO) {
                    currentState = ShootState.PRONTO_PARA_ATIRAR;
                    stateTimer.reset();
                }
                break;

            case PRONTO_PARA_ATIRAR:
                // Aguarda estabilização
                if (stateTimer.seconds() >= TEMPO_PARADO_ATIRAR) {
                    // Permanece aqui até comecarATirar() ser chamado
                }
                break;

            case ATIRANDO_BOLINHA_1:
                // Primeira bolinha: liga feeder
                ligarFeeder();

                if (stateTimer.seconds() >= TEMPO_FEEDER_BOLINHA) {
                    bolinhasAtiradas = 1;
                    currentState = ShootState.ESPERANDO_1_2;
                    stateTimer.reset();
                }
                break;

            case ESPERANDO_1_2:
                // Mantém feeder ligado durante espera
                if (stateTimer.seconds() >= TEMPO_ENTRE_BOLINHAS) {
                    currentState = ShootState.ATIRANDO_BOLINHA_2;
                    stateTimer.reset();
                }
                break;

            case ATIRANDO_BOLINHA_2:
                // Segunda bolinha: feeder já está ligado
                if (stateTimer.seconds() >= TEMPO_FEEDER_BOLINHA) {
                    bolinhasAtiradas = 2;
                    currentState = ShootState.ESPERANDO_2_3;
                    stateTimer.reset();
                }
                break;

            case ESPERANDO_2_3:
                // Mantém feeder ligado durante espera
                if (stateTimer.seconds() >= TEMPO_ENTRE_BOLINHAS) {
                    currentState = ShootState.PREPARANDO_BOLINHA_3;
                    stateTimer.reset();
                }
                break;

            case PREPARANDO_BOLINHA_3:
                // Terceira bolinha: sobe servo (feeder continua ligado)
                if (alavancaServo != null) {
                    alavancaServo.setPosition(POSICAO_SERVO_ATIRAR);
                }

                if (stateTimer.seconds() >= TEMPO_SERVO_ACIONA) {
                    currentState = ShootState.ATIRANDO_BOLINHA_3;
                    stateTimer.reset();
                }
                break;

            case ATIRANDO_BOLINHA_3:
                // Continua alimentando terceira bolinha
                if (stateTimer.seconds() >= TEMPO_FEEDER_BOLINHA) {
                    currentState = ShootState.FINALIZANDO;
                    stateTimer.reset();
                }
                break;

            case FINALIZANDO:
                // Desliga tudo
                desligarFeeder();
                desligarShooter();

                // Volta servo para repouso
                if (alavancaServo != null) {
                    alavancaServo.setPosition(POSICAO_SERVO_REPOUSO);
                }

                bolinhasAtiradas = 3;
                cicloEmAndamento = false;
                currentState = ShootState.IDLE;
                break;
        }
    }

    /**
     * Começa a sequência de tiro
     */
    public void comecarATirar() {
        if (currentState == ShootState.PRONTO_PARA_ATIRAR) {
            currentState = ShootState.ATIRANDO_BOLINHA_1;
            stateTimer.reset();
            ligarFeeder(); // Liga feeder ao começar
        }
    }

    /**
     * Para emergência: desliga tudo imediatamente
     */
    public void emergencyStop() {
        desligarFeeder();
        desligarShooter();

        if (alavancaServo != null) {
            alavancaServo.setPosition(POSICAO_SERVO_REPOUSO);
        }

        currentState = ShootState.IDLE;
        cicloEmAndamento = false;
    }

    // ========== MÉTODOS DE CONSULTA ==========

    public ShootState getEstadoAtual() {
        return currentState;
    }

    public boolean isIdle() {
        return currentState == ShootState.IDLE;
    }

    public boolean isReadyToShoot() {
        return currentState == ShootState.PRONTO_PARA_ATIRAR;
    }

    public boolean isShooting() {
        return currentState != ShootState.IDLE;
    }

    public int getBolinhasAtiradas() {
        return bolinhasAtiradas;
    }

    public boolean isCicloEmAndamento() {
        return cicloEmAndamento;
    }

    public double getTempoEstadoAtual() {
        return stateTimer != null ? stateTimer.seconds() : 0.0;
    }

    /**
     * @return tempo que o shooter está ligado
     */
    public double getTempoShooterLigado() {
        return shooterTimer.seconds();
    }

    /**
     * @return true se o shooter está ligado
     */
    public boolean isShooterLigado() {
        return shooterLigado;
    }

    /**
     * @return true se o feeder está ligado
     */
    public boolean isFeederLigado() {
        return feederLigado;
    }
}