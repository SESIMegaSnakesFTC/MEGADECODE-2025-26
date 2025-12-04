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
    private DcMotor motor;
    private DcMotor motorExtra;

    private CRServo servoE;
    private CRServo servoD;

    private enum LastSeenDirection {LEFT, RIGHT, NONE}

    private LastSeenDirection lastSeen = LastSeenDirection.NONE;
    private boolean modoAutomatico = false;
    private boolean lastTriggerPressed = false;

    private boolean motorExtraLigado = false;
    private boolean ultimoA = false;

    private static final double KP = 0.02;
    private static final double SEARCH_SPEED = 0.35;

    @Override
    public void init() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        motor = hardwareMap.get(DcMotor.class, "motor");
        motorExtra = hardwareMap.get(DcMotor.class, "motorExtra");
        servoE = hardwareMap.get(CRServo.class, "servoE");
        servoD = hardwareMap.get(CRServo.class, "servoD");

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorExtra.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        limelight.start();
        limelight.pipelineSwitch(0); // VERMELHO
    }

    @Override
    public void loop() {

        boolean rtPressed = gamepad1.right_trigger > 0.8;
        if (rtPressed && !lastTriggerPressed) {
            modoAutomatico = !modoAutomatico;
        }
        lastTriggerPressed = rtPressed;

        double stickY = -gamepad1.left_stick_y;
        double manualSpeedLimit = 0.5;
        double manualPower = gamepad1.left_stick_x * manualSpeedLimit;

        servoE.setPower(manualPower);
        servoD.setPower(-manualPower);

        boolean botaoA = gamepad1.a;
        if (gamepad1.a && !ultimoA) {
            motorExtraLigado = !motorExtraLigado; // alterna estado
        }
        ultimoA = botaoA;

        motorExtra.setPower(motorExtraLigado ? 1.0 : 0.0);

        // ====== Seleciona modo de operação ======
        if (modoAutomatico) {
            runAutomaticMode();
        } else {
            runManualMode();
        }

        // ====== Telemetria resumida ======
        telemetry.addData("Modo", modoAutomatico ? "AUTOMÁTICO" : "MANUAL");
        telemetry.addData("RT pressionado", rtPressed);
        telemetry.addData("CRServo1 Power", "%.3f", servoE.getPower());
        telemetry.addData("CRServo2 Power", "%.3f", servoD.getPower());
        telemetry.addData("MotorExtra", motorExtraLigado ? "LIGADO" : "DESLIGADO");
        telemetry.update();
    }
    private void runManualMode() {
        //Modo manual
        motor.setPower(gamepad1.left_stick_x * 0.4);
    }

    private void runAutomaticMode() {
        //Modo automático
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            //Limelight de ponta cabeça
            double rawTx = result.getTx();
            double rawTy = result.getTy();
            double tx = -rawTx;
            double ty = -rawTy;

            if (tx > 0) lastSeen = LastSeenDirection.RIGHT;
            else if (tx < 0) lastSeen = LastSeenDirection.LEFT;

            // PID
            double correction = tx * KP;
            if (correction > 0.4) correction = 0.4;
            if (correction < -0.4) correction = -0.4;

            motor.setPower(correction);

            telemetry.addLine("=== MODO AUTOMÁTICO ===");
            telemetry.addData("Status", "Tag Detectada");
            telemetry.addData("Tx (corrigido)", "%.3f", tx);
            telemetry.addData("Ty (corrigido)", "%.3f", ty);
            telemetry.addData("Correção Motor", "%.3f", correction);
            telemetry.addData("Última direção", lastSeen.toString());

        } else {
            telemetry.addLine("=== MODO AUTOMÁTICO ===");
            telemetry.addData("Status", "Tag Perdida");

            // Busca baseada na última direção conhecida
            switch (lastSeen) {
                case LEFT:
                    motor.setPower(SEARCH_SPEED);
                    telemetry.addData("Buscando", "Esquerda");
                    break;
                case RIGHT:
                    motor.setPower(-SEARCH_SPEED);
                    telemetry.addData("Buscando", "Direita");
                    break;
                case NONE:
                    motor.setPower(SEARCH_SPEED);
                    telemetry.addData("Buscando", "Padrão (direita)");
                    break;
            }
        }
    }
}
