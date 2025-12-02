package org.firstinspires.ftc.teamcode.testesMecanismos;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@Disabled
@TeleOp(name = "limelightv3")
public class limelightPIDv3 extends OpMode {

    private Limelight3A limelight;
    private DcMotor motorTurret;

    // PARÂMETROS PD
    private double kP = 0.08;
    private double kD = 0.00; // começar com 0.0; aumente com cuidado

    // LIMITES E AJUSTES
    private double MAX_POWER = 0.4;
    private double MIN_POWER = 0.05;   // potência mínima para vencer atrito
    private double DEADBAND_DEG = 0.2; // zona morta em graus
    private double SMOOTH_ALPHA = 0.25; // suavização exponencial (0..1) — maior = mais responsivo
    private double SEARCH_POWER = 0.12; // potência de busca
    private long SEARCH_SWEEP_MS = 1500; // tempo entre inversões de sweep quando sem alvo

    // VARIÁVEIS INTERNAS
    private double smoothedPower = 0.0;
    private double lastErrorX = 0.0;
    private long lastTimeMs = 0;
    private long lastSearchSwitch = 0;
    private boolean tagVisible = false;
    private boolean searchingRight = true;

    // MODO DE OPERAÇÃO
    private boolean modoAutomatico = false;
    private boolean lastTriggerPressed = false;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        motorTurret = hardwareMap.get(DcMotor.class, "motor");

        motorTurret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorTurret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight.pipelineSwitch(0);
        limelight.start();
        limelight.setPollRateHz(100);

        telemetry.addLine("Sistema Limelight PD pronto.");
        telemetry.update();
    }

    @Override
    public void start() {
        lastTimeMs = System.currentTimeMillis();
        lastSearchSwitch = lastTimeMs;
    }

    @Override
    public void loop() {

        boolean triggerPressed = gamepad1.right_trigger > 0.6;

        // Alterna entre manual e automático com o gatilho direito (na transição)
        if (triggerPressed && !lastTriggerPressed) {
            modoAutomatico = !modoAutomatico;
        }
        lastTriggerPressed = triggerPressed;

        if (modoAutomatico) {
            modoAutomaticoLoop();
        } else {
            executarModoManual();
        }

        telemetry.addLine("=== CONTROLE LIMELIGHT ===");
        telemetry.addData("Modo", modoAutomatico ? "Automático (PD)" : "Manual");
        telemetry.addData("RT pressionado", triggerPressed);
        telemetry.update();
    }

    private void modoAutomaticoLoop() {
        LLResult result = limelight.getLatestResult();
        double erroX = 0.0;
        boolean currentTagVisible = false;

        // Lê dados da Limelight
        if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {
            LLResultTypes.FiducialResult fr = result.getFiducialResults().get(0);
            erroX = fr.getTargetXDegrees();
            currentTagVisible = true;
        }

        long now = System.currentTimeMillis();
        double dt = (now - lastTimeMs) / 1000.0;
        if (dt <= 0.0) dt = 1e-3; // proteção contra dt zero
        lastTimeMs = now;

        double commandedPower = 0.0;

        if (currentTagVisible) {
            // Temos alvo visível -> controle PD
            tagVisible = true;

            if (Math.abs(erroX) <= DEADBAND_DEG) {
                commandedPower = 0.0;
            } else {
                double pTerm = erroX * kP;
                double dTerm = ((erroX - lastErrorX) / dt) * kD; // derivada
                commandedPower = pTerm + dTerm;

                // Clip para limites máximos
                commandedPower = Range.clip(commandedPower, -MAX_POWER, MAX_POWER);

                // Garantir potência mínima para vencer atrito (se não zero)
                if (Math.abs(commandedPower) > 0.0 && Math.abs(commandedPower) < MIN_POWER) {
                    commandedPower = Math.signum(commandedPower) * MIN_POWER;
                }
            }
        } else {
            // Alvo perdido -> estratégia de busca inteligente
            if (tagVisible) {
                // tivemos o alvo antes, então iniciamos busca mantendo sentido que tínhamos
                // se lastErrorX ≈ 0, mantemos searchingRight como estava; caso contrário, baseamos no sinal do lastErrorX
                if (Math.abs(lastErrorX) > 0.5) {
                    searchingRight = lastErrorX > 0.0;
                }
                tagVisible = false;
                lastSearchSwitch = now; // reinicia timer de sweep
            }

            // A cada SEARCH_SWEEP_MS invertemos o sentido para varrer a área
            if (now - lastSearchSwitch >= SEARCH_SWEEP_MS) {
                searchingRight = !searchingRight;
                lastSearchSwitch = now;
            }

            commandedPower = searchingRight ? SEARCH_POWER : -SEARCH_POWER;
        }

        // Suavização exponencial
        smoothedPower = SMOOTH_ALPHA * commandedPower + (1.0 - SMOOTH_ALPHA) * smoothedPower;

        // Aplicar potência ao motor
        motorTurret.setPower(smoothedPower);

        // Telemetria
        telemetry.addLine("=== MODO AUTOMÁTICO ===");
        telemetry.addData("Erro X (°)", "%.2f", erroX);
        telemetry.addData("Potência (comando)", "%.3f", commandedPower);
        telemetry.addData("Potência (suavizada)", "%.3f", smoothedPower);
        telemetry.addData("Tag visível", currentTagVisible);
        telemetry.addData("Busca sentido", searchingRight ? "→ Direita" : "← Esquerda");
        telemetry.addData("kP", kP);
        telemetry.addData("kD", kD);

        lastErrorX = erroX;
    }

    // MODO MANUAL
    private void executarModoManual() {
        double input = gamepad1.right_stick_x;
        double motorPower = Range.clip(input, -MAX_POWER, MAX_POWER);

        motorTurret.setPower(motorPower);

        telemetry.addLine("=== MODO MANUAL ===");
        telemetry.addData("Power Motor", "%.2f", motorPower);
    }

    @Override
    public void stop() {
        limelight.stop();
        motorTurret.setPower(0);
    }
}
