package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.Locale;
@Disabled

public class PaineldeObservacaoAZUL {
    public static void atualizarTelemetriaReconhecido (Telemetry telemetry, LLStatus statusLimelight, LLResult resultadoLimeLight, double distancia){
        telemetry.addLine("PAINEL DE OBSERVAÇÃO: AZUL");
        telemetry.addLine();
        telemetry.addLine("HARDWARE: Limelight");
        telemetry.addLine(String.format(Locale.US, "Temperatura: %.1f°C | FPS: %d | CPU: %.1f%%", statusLimelight.getTemp(), (int) statusLimelight.getFps(), statusLimelight.getCpu()));
        telemetry.addLine();
        telemetry.addLine("CONFIGURAÇÃO: Limelight");
        telemetry.addData("Pipeline:", "Index: %d | Tipo: %s", statusLimelight.getPipelineIndex(), statusLimelight.getPipelineType().toString());
        telemetry.addLine();
        telemetry.addLine("STATUS: Limelight");
        telemetry.addData("Distância do Alvo", "%.1f cm", distancia);
        telemetry.addData("Tx:", "%f°", resultadoLimeLight.getTx());
        telemetry.addData("Ty:", "%f°", resultadoLimeLight.getTy());
        telemetry.addData("Ta:", "%fcm²", resultadoLimeLight.getTa());
    }

    public static void atualizarTelemetriaProcurando (Telemetry telemetry, LLStatus statusLimelight){
        telemetry.addLine("PAINEL DE OBSERVAÇÃO:");
        telemetry.addLine();
        telemetry.addLine("HARDWARE: Limelight");
        telemetry.addLine(String.format(Locale.US, "Temperatura: %.1f°C | FPS: %d | CPU: %.1f%%", statusLimelight.getTemp(), (int) statusLimelight.getFps(), statusLimelight.getCpu()));
        telemetry.addLine();
        telemetry.addLine("CONFIGURAÇÃO: Limelight");
        telemetry.addData("Pipeline:", "Index: %d | Tipo: %s", statusLimelight.getPipelineIndex(), statusLimelight.getPipelineType().toString());
        telemetry.addLine();
        telemetry.addLine("STATUS: Limelight");
        telemetry.addLine("PROCURANDO APRILTAG...");
        telemetry.addLine();
    }
}