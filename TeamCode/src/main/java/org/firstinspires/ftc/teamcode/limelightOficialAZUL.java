package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

@Disabled
@TeleOp(name = "Limelight Oficial AZUL")
public class limelightOficialAZUL extends OpMode {

    private Limelight3A limelight;
    private DcMotor motor;

    private enum LastSeenDirection {LEFT, RIGHT, NONE}

    private LastSeenDirection lastSeen = LastSeenDirection.NONE;
    private boolean modoAutomatico = false;
    private boolean lastTriggerPressed = false;

    private static final double KP = 0.02;
    private static final double SEARCH_SPEED = 0.15;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        motor = hardwareMap.get(DcMotor.class, "motor");

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight.start();
        limelight.pipelineSwitch(1); //PIPELINE AZUL
    }

    @Override
    public void loop() {

        boolean rtPressed = gamepad1.right_trigger > 0.6;

        if (rtPressed && !lastTriggerPressed) {
            modoAutomatico = !modoAutomatico;
        }

        lastTriggerPressed = rtPressed;

        if (modoAutomatico) {
            runAutomaticMode();
        } else {
            runManualMode();
        }

        telemetry.addData("Modo", modoAutomatico ? "AUTOMÁTICO" : "MANUAL");
        telemetry.addData("RT pressionado", rtPressed);
    }

    private void runManualMode() {
        double manualPower = gamepad1.left_stick_x;
        motor.setPower(manualPower);

        telemetry.addLine("=== MODO MANUAL ===");
        telemetry.addData("Controle", "Use o stick esquerdo horizontal");
        telemetry.addData("Potência", manualPower);
    }

    private void runAutomaticMode() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double tx = result.getTx();

            if (tx > 0) lastSeen = LastSeenDirection.RIGHT;
            else if (tx < 0) lastSeen = LastSeenDirection.LEFT;

            double correction = -tx * KP;
            correction = Math.max(-0.4, Math.min(0.4, correction));

            motor.setPower(correction);

            telemetry.addLine("=== MODO AUTOMÁTICO ===");
            telemetry.addData("Status", "Tag Detectada");
            telemetry.addData("Tx", tx);
            telemetry.addData("Motor", correction);
            telemetry.addData("Última direção", lastSeen);

        } else {
            telemetry.addLine("=== MODO AUTOMÁTICO ===");
            telemetry.addData("Status", "Tag Perdida");

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
