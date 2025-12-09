package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

@TeleOp(name = "Limelight Oficial VERMELHA 1")
public class limelightOficialVERMELHO extends OpMode {

    private Limelight3A limelight;
    private DcMotor baseShooter;
    private DcMotor shooter;

    private CRServo servoE;   // Servo contínuo
    private CRServo servoD;   // Servo contínuo

    private enum LastSeenDirection {LEFT, RIGHT, NONE}

    private LastSeenDirection lastSeen = LastSeenDirection.NONE;
    private boolean modoAutomatico = false;
    private boolean lastTriggerPressed = false;

    private boolean shooterLigado = false;
    private boolean ultimoA = false;

    private static final double KP = 0.02;
    private static final double SEARCH_SPEED = 0.35;

    @Override
    public void init() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        baseShooter = hardwareMap.get(DcMotor.class, "baseShooter");
        shooter = hardwareMap.get(DcMotor.class, "shooter");

        servoE = hardwareMap.get(CRServo.class, "servoE"); // contínuo
        servoD = hardwareMap.get(CRServo.class, "servoD"); // contínuo invertido

        baseShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        limelight.start();
        limelight.pipelineSwitch(0); // VERMELHO

        // Reset inicial — ambos parados
        servoE.setPower(0);
        servoD.setPower(0);
    }

    @Override
    public void loop() {

        // Alternar modo automático
        boolean rtPressed = gamepad1.right_trigger > 0.8;
        if (rtPressed && !lastTriggerPressed) {
            modoAutomatico = !modoAutomatico;
        }
        lastTriggerPressed = rtPressed;

        // Alternar shooter
        boolean botaoA = gamepad1.a;
        if (botaoA && !ultimoA) {
            shooterLigado = !shooterLigado;
        }
        ultimoA = botaoA;

        shooter.setPower(shooterLigado ? 1.0 : 0.0);

        // Executa modo
        if (modoAutomatico) runAutomaticMode();
        else runManualMode();

        // Telemetria
        telemetry.addData("Modo", modoAutomatico ? "AUTOMÁTICO" : "MANUAL");
        telemetry.addData("ServoE Power", servoE.getPower());
        telemetry.addData("ServoD Power", servoD.getPower());
        telemetry.addData("Shooter", shooterLigado ? "LIGADO" : "DESLIGADO");
        telemetry.update();
    }

    // =====================================================
    // ===============     MODO MANUAL      ================
    // =====================================================

    private void runManualMode() {

        // Movimento horizontal
        baseShooter.setPower(gamepad1.left_stick_x * 0.35);

        double stickY = -gamepad1.right_stick_y;  // Para cima = positivo

        // Dois servos contínuos trabalhando juntos
        // Servo E normal
        servoE.setPower(stickY);

        // Servo D invertido
        servoD.setPower(-stickY);
    }

    // =====================================================
    // ===============   MODO AUTOMÁTICO    ================
    // =====================================================

    private void runAutomaticMode() {

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            double rawTx = result.getTx();
            double tx = -rawTx;

            if (tx > 0) lastSeen = LastSeenDirection.RIGHT;
            else if (tx < 0) lastSeen = LastSeenDirection.LEFT;

            double correction = tx * KP;
            correction = Math.max(-0.4, Math.min(0.4, correction));

            baseShooter.setPower(correction);

            telemetry.addLine("=== AUTOMÁTICO ===");
            telemetry.addData("Status", "Tag Detectada");
            telemetry.addData("Tx", tx);

        } else {

            telemetry.addLine("=== AUTOMÁTICO ===");
            telemetry.addData("Status", "Tag Perdida");

            switch (lastSeen) {

                case LEFT:
                    baseShooter.setPower(SEARCH_SPEED);
                    telemetry.addData("Buscando", "Esquerda");
                    break;

                case RIGHT:
                    baseShooter.setPower(-SEARCH_SPEED);
                    telemetry.addData("Buscando", "Direita");
                    break;

                case NONE:
                    baseShooter.setPower(SEARCH_SPEED);
                    telemetry.addData("Buscando", "Padrão");
                    break;
            }
        }
    }
}
