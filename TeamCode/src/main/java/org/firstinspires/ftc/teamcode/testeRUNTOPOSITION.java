package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
// Mude para @Autonomous se for usar em modo autônomo
@Autonomous(name = "RUN TO POSITION", group = "FTC")
public class testeRUNTOPOSITION extends LinearOpMode {

    private DcMotor baseShooter = null;

    // Constantes do Motor
    // NOVO CÁLCULO: 28 ticks/rev do motor * (4:1 * 5:1) = 560 ticks/rev do eixo de saída
    static final double TICKS_POR_REVOLUCAO = 560.0;
    static final double ROTACAO_ALVO = -0.54;         // Posição desejada (
    static final double POWER = 0.5;                // Potência para o movimento

    @Override
    public void runOpMode() {

        // 1. Mapeamento do Motor
        baseShooter = hardwareMap.get(DcMotor.class, "baseShooter");

        // ... (o restante do seu código permanece o mesmo)

        // **FASE 1: Começar sempre no Ponto 0**
        baseShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        baseShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        baseShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        baseShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Pronto e Encoder Zerado");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            // **FASE 2: Ir para a Posição 3.0 Rotações**

            // A. Cálculo: Ticks = Rotações * Ticks/Rev
            int posicaoAlvoTicks = (int) (ROTACAO_ALVO * TICKS_POR_REVOLUCAO);
            // Com os novos valores: 3.0 * 560.0 = 1680 Ticks

            // B. Definir o Alvo
            baseShooter.setTargetPosition(posicaoAlvoTicks);

            // C. Ativar o Modo de Controle de Posição
            baseShooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // D. Iniciar o Movimento
            baseShooter.setPower(POWER);

            // E. Esperar o Motor Chegar à Posição
            while (opModeIsActive() && baseShooter.isBusy()) {
                telemetry.addData("Status", "Movendo...");
                telemetry.addData("Alvo (Ticks)", posicaoAlvoTicks);
                telemetry.addData("Atual (Ticks)", baseShooter.getCurrentPosition());
                telemetry.update();
                idle(); // Permite que outras threads do FTC rodem
            }

            // F. Parar
            baseShooter.setPower(0);

            // G. Retornar ao modo padrão
            baseShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.addData("Status", "Chegou a %s Rotações.", ROTACAO_ALVO);
            telemetry.addData("Posição Final", baseShooter.getCurrentPosition());
            telemetry.update();

            // Permanece no loop até o fim do OpMode
            while (opModeIsActive()) {
                idle();
            }
        }
    }
}