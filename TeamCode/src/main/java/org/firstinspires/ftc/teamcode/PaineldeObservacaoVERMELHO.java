package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

@Disabled
public class PaineldeObservacaoVERMELHO {

    public static void atualizarTelemetriaReconhecido(
            Telemetry telemetry,
            LLStatus statusLimelight,
            LLResult resultadoLimeLight,
            double velocidadeChassi,
            String statusFeeder,
            Follower follower,
            String estadoPathing
    ) {

        telemetry.addLine("PAINEL DE OBSERVAÇÃO: VERMELHO");
        telemetry.addLine();

        telemetry.addLine("CONFIGURAÇÃO: Limelight");
        telemetry.addLine(String.format(Locale.US,
                "Temp: %.1f°C | FPS: %d | CPU: %.1f%%",
                statusLimelight.getTemp(),
                (int) statusLimelight.getFps(),
                statusLimelight.getCpu()));
        telemetry.addData("Pipeline:", "Index: %d | Tipo: %s",
                statusLimelight.getPipelineIndex(),
                statusLimelight.getPipelineType().toString());

        telemetry.addLine();
        telemetry.addLine("STATUS: Limelight");
        telemetry.addData("Tx:", "%f°", resultadoLimeLight.getTx());
        telemetry.addData("Ty:", "%f°", resultadoLimeLight.getTy());
        telemetry.addData("Ta:", "%fcm²", resultadoLimeLight.getTa());

        telemetry.addLine();
        telemetry.addLine("CONFIGURAÇÃO: Geral");
        telemetry.addData("Velocidade", "%.3f", velocidadeChassi);
        telemetry.addData("Feeder", statusFeeder);
        telemetry.addData("Caminho: ", estadoPathing);
        telemetry.addData("x: ", "%.2f°", follower.getPose().getX());
        telemetry.addData("y: ", "%.2f°", follower.getPose().getY());
        telemetry.addData("Direção: ", "%.2f°", follower.getPose().getHeading());
        telemetry.addData("Seguindo caminho?", follower.isBusy());
    }

    public static void atualizarTelemetriaProcurando(
            Telemetry telemetry,
            LLStatus statusLimelight,
            LLResult resultadoLimeLight,
            double velocidadeChassi,
            String statusFeeder,
            Follower follower,
            String estadoPathing
    ) {

        telemetry.addLine("PAINEL DE OBSERVAÇÃO: VERMELHO");
        telemetry.addLine();

        telemetry.addLine("CONFIGURAÇÃO: Limelight");
        telemetry.addLine(String.format(Locale.US,
                "Temp: %.1f°C | FPS: %d | CPU: %.1f%%",
                statusLimelight.getTemp(),
                (int) statusLimelight.getFps(),
                statusLimelight.getCpu()));
        telemetry.addData("Pipeline:", "Index: %d | Tipo: %s",
                statusLimelight.getPipelineIndex(),
                statusLimelight.getPipelineType().toString());

        telemetry.addLine();
        telemetry.addLine("STATUS: Limelight");
        telemetry.addData("Tx:", "%f°", resultadoLimeLight.getTx());
        telemetry.addData("Ty:", "%f°", resultadoLimeLight.getTy());
        telemetry.addData("Ta:", "%fcm²", resultadoLimeLight.getTa());
        telemetry.addData("Distância do Alvo", "%.1f cm");

        telemetry.addLine();
        telemetry.addLine("CONFIGURAÇÃO: Geral");
        telemetry.addData("Velocidade", "%.3f", velocidadeChassi);
        telemetry.addData("Feeder", statusFeeder);
        telemetry.addData("Caminho: ", estadoPathing);
        telemetry.addData("x: ", "%.2f°", follower.getPose().getX());
        telemetry.addData("y: ", "%.2f°", follower.getPose().getY());
        telemetry.addData("Direção: ", "%.2f°", follower.getPose().getHeading());
        telemetry.addData("Seguindo caminho?", follower.isBusy());
    }
}
