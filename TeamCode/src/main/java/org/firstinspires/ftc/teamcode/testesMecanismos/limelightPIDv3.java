package org.firstinspires.ftc.teamcode.testesMecanismos;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "limelightv3")
public class limelightPIDv3 extends OpMode {

    private Limelight3A limelight;
    private DcMotor motorTurret;

    // PARÂMETROS DO PD
    private double kP = 0.08;
    private double kD = 0.00;

    // LIMITES E AJUSTES
    private double MAX_POWER = 0.4;
    private double MIN_POWER = 0.05;
    private double DEADBAND_DEG = 0.2;
    private double SMOOTH_ALPHA = 0.1;  // suavização
    private double SEARCH_POWER = 0.1;

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

    // DISTÂNCIA
    private double distance;


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
    }

    @Override
    public void start() {
        lastTimeMs = System.currentTimeMillis();
    }

    @Override
    public void loop() {

        boolean triggerPressed = gamepad1.right_trigger > 0.6;

        // alterna entre manual e automático com o gatilho direito
        if (triggerPressed && !lastTriggerPressed) {
            modoAutomatico = !modoAutomatico;
        }
        lastTriggerPressed = triggerPressed;

        if (modoAutomatico) {
            EXmodoAutomatico();
        } else {
            executarModoManual();
        }

        telemetry.addLine("=== CONTROLE LIMELIGHT ===");
        telemetry.addData("Modo", modoAutomatico ? "Automático (PD)" : "Manual");
        telemetry.addData("RT pressionado", triggerPressed);
        telemetry.update();
    }


    private void EXmodoAutomatico() {
        LLResult result = limelight.getLatestResult();
        double erroX = 0.0;
        boolean currentTagVisible = false;

        // --- Lê dados da Limelight ---
        if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {
            LLResultTypes.FiducialResult fr = result.getFiducialResults().get(0);
            erroX = fr.getTargetXDegrees();
            currentTagVisible = true;
        }

        long now = System.currentTimeMillis();
        double dt = Math.max((now - lastTimeMs) / 1000.0, 1e-3);
        lastTimeMs = now;

        double commandedPower;

        if (currentTagVisible) {
            tagVisible = true;

            // Se o erro estiver dentro da zona morta, não corrige
            if (Math.abs(erroX) <= DEADBAND_DEG) {
                commandedPower = 0.0;
            } else {
                // Controle PD
                double pTerm = erroX * kP;
                double dTerm = ((erroX - lastErrorX) / dt) * kD;
                commandedPower = pTerm + dTerm;

                commandedPower = Range.clip(commandedPower, (-MAX_POWER + -MIN_POWER)/2, (MAX_POWER + MIN_POWER)/2);

                // Potência mínima para vencer atrito
                if (commandedPower != 0.0 && Math.abs(commandedPower) < MIN_POWER) {
                    commandedPower = Math.signum(commandedPower) * MIN_POWER;
                }
            }

        } else {
            // --- TAG PERDIDA: busca inteligente ---
            if (tagVisible) {
                // Decide o sentido da busca baseado no último erro
                searchingRight = lastErrorX > 0;
                tagVisible = false;
            }

            // Mantém a busca até encontrar novamente
            commandedPower = searchingRight ? SEARCH_POWER : -SEARCH_POWER;
        }

        // Suavização de potência
        smoothedPower = SMOOTH_ALPHA * commandedPower + (1.0 - SMOOTH_ALPHA) * smoothedPower;

        // Aplica no motor
        motorTurret.setPower(smoothedPower);

        // --- Telemetria ---
        telemetry.addLine("=== MODO AUTOMÁTICO ===");
        telemetry.addData("Erro X (°)", "%.2f", erroX);
        telemetry.addData("Potência", "%.3f", smoothedPower);
        telemetry.addData("Tag visível", currentTagVisible);
        telemetry.addData("Busca", searchingRight ? "→ Direita" : "← Esquerda");
        telemetry.addData("kP", kP);
        telemetry.addData("kD", kD);


        lastErrorX = erroX;
    }

    // MODO MANUAL
    private void executarModoManual() {
        double input = gamepad1.right_stick_x;
        double motorPower = Range.clip(input, -0.4, 0.4);

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
