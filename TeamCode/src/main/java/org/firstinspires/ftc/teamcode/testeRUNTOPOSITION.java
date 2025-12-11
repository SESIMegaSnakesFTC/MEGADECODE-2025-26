package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

// Mude para @Autonomous se for usar em modo autônomo
@Autonomous(name = "RUN TO POSITION", group = "FTC")
public class testeRUNTOPOSITION extends LinearOpMode {

    private DcMotor motor = null;

    // Constantes do Motor
    static final double TICKS_POR_REVOLUCAO = 28.0; // HD Hex Motor sem redução
    static final double ROTACAO_ALVO = 0.8;         // Posição desejada (0.8 voltas)
    static final double POWER = 0.5;                // Potência para o movimento

    @Override
    public void runOpMode() {

        // 1. Mapeamento do Motor
        // Substitua "motor_hdhex" pelo nome configurado no Driver Station
        motor = hardwareMap.get(DcMotor.class, "motor_hdhex");

        // 2. Configurações e Zero
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // **FASE 1: Começar sempre no Ponto 0**
        // Zera o contador do encoder na posição atual
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Prepara para usar o encoder em modo de velocidade/posição após o reset
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Pronto e Encoder Zerado");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            // **FASE 2: Ir para a Posição 0.8**

            // A. Cálculo: Ticks = Rotações * Ticks/Rev
            int posicaoAlvoTicks = (int) (ROTACAO_ALVO * TICKS_POR_REVOLUCAO);

            // B. Definir o Alvo
            motor.setTargetPosition(posicaoAlvoTicks);

            // C. Ativar o Modo de Controle de Posição
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // D. Iniciar o Movimento
            motor.setPower(POWER);

            // E. Esperar o Motor Chegar à Posição
            while (opModeIsActive() && motor.isBusy()) {
                telemetry.addData("Status", "Movendo...");
                telemetry.addData("Alvo (Ticks)", posicaoAlvoTicks);
                telemetry.addData("Atual (Ticks)", motor.getCurrentPosition());
                telemetry.update();
                idle(); // Permite que outras threads do FTC rodem
            }

            // F. Parar (Opcional, mas recomendado para garantir que pare de aplicar força)
            motor.setPower(0);

            // G. Retornar ao modo padrão (opcional)
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.addData("Status", "Chegou à 0.8 Rotações.");
            telemetry.addData("Posição Final", motor.getCurrentPosition());
            telemetry.update();

            // Permanece no loop até o fim do OpMode
            while (opModeIsActive()) {
                idle();
            }
        }
    }
}