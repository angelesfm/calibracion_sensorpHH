/**
 * @file main.c
 * @brief Probador simple para módulo de pH PH-4502C con ESP32.
 *
 * Este firmware lee la salida analógica del módulo pH conectada al GPIO34,
 * calcula el voltaje y aplica una ecuación lineal:
 *
 * pH = PH_SLOPE * voltaje + PH_OFFSET
 *
 * Uso:
 * - PO del módulo pH -> GPIO34, directamente solo si PO <= 3.3 V.
 * - G del módulo pH  -> GND ESP32.
 * - V+ del módulo pH -> 5 V, si tu módulo trabaja a 5 V.
 *
 * Si PO puede superar 3.3 V, usa divisor de voltaje antes del GPIO34.
 */

#include <stdio.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

/* =========================
 * Configuración del ADC
 * ========================= */

#define PH_ADC_UNIT             ADC_UNIT_1
#define PH_ADC_CHANNEL          ADC_CHANNEL_6      /* GPIO34 */
#define PH_ADC_ATTEN            ADC_ATTEN_DB_12
#define PH_ADC_BITWIDTH         ADC_BITWIDTH_DEFAULT

#define ADC_AVG_SAMPLES         20
#define SAMPLE_DELAY_MS         10
#define LOOP_DELAY_MS           1000

/* =========================
 * Calibración de pH
 * =========================
 *
 * Estos son los valores que ya tenía tu código originapl.
 * Luego los puedes cambiar por los valores calculados con tus buffers.
 */
#define PH_SLOPE                (-5.70f)
#define PH_OFFSET               (21.34f)

/* =========================
 * Divisor de voltaje
 * =========================
 *
 * Si conectas PO directo a GPIO34:
 *   DIVIDER_RATIO = 1.0f
 *
 * Si usas divisor:
 *
 *   PO --- R1 --- GPIO34 --- R2 --- GND
 *
 * entonces:
 *   DIVIDER_RATIO = (R1 + R2) / R2
 *
 * Ejemplo:
 *   R1 = 10k
 *   R2 = 15k
 *   DIVIDER_RATIO = 25k / 15k = 1.6667
 *
 * IMPORTANTE:
 * Para calcular PH_SLOPE y PH_OFFSET usa preferiblemente V_GPIO34,
 * es decir, el voltaje que realmente lee la ESP32.
 */
#define DIVIDER_RATIO           (1.0f)

static const char *TAG = "PH_TESTER";

static adc_oneshot_unit_handle_t adc_handle = NULL;
static adc_cali_handle_t adc_cali_handle = NULL;
static bool adc_calibrated = false;

/**
 * @brief Inicializa la calibración interna del ADC.
 */
static void adc_calibration_init(void)
{
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = PH_ADC_UNIT,
        .atten = PH_ADC_ATTEN,
        .bitwidth = PH_ADC_BITWIDTH
    };

    esp_err_t ret = adc_cali_create_scheme_line_fitting(
        &cali_config,
        &adc_cali_handle
    );

    if (ret == ESP_OK) {
        adc_calibrated = true;
        ESP_LOGI(TAG, "Calibracion ADC habilitada");
    } else {
        adc_calibrated = false;
        ESP_LOGW(TAG, "No se pudo habilitar calibracion ADC: %s", esp_err_to_name(ret));
    }
}

/**
 * @brief Convierte lectura ADC cruda a voltaje.
 */
static float adc_raw_to_voltage(int raw)
{
    if (adc_calibrated && adc_cali_handle != NULL) {
        int voltage_mv = 0;

        esp_err_t ret = adc_cali_raw_to_voltage(
            adc_cali_handle,
            raw,
            &voltage_mv
        );

        if (ret == ESP_OK) {
            return voltage_mv / 1000.0f;
        }
    }

    /*
     * Respaldo aproximado si la calibración ADC no está disponible.
     */
    return ((float) raw / 4095.0f) * 3.3f;
}

/**
 * @brief Inicializa ADC1 canal 6, que corresponde a GPIO34.
 */
static esp_err_t ph_adc_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = PH_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };

    esp_err_t ret = adc_oneshot_new_unit(&unit_cfg, &adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo inicializar ADC: %s", esp_err_to_name(ret));
        return ret;
    }

    adc_oneshot_chan_cfg_t channel_cfg = {
        .atten = PH_ADC_ATTEN,
        .bitwidth = PH_ADC_BITWIDTH
    };

    ret = adc_oneshot_config_channel(
        adc_handle,
        PH_ADC_CHANNEL,
        &channel_cfg
    );

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo configurar GPIO34/ADC1_CH6: %s", esp_err_to_name(ret));
        return ret;
    }

    adc_calibration_init();

    return ESP_OK;
}

/**
 * @brief Lee varias muestras y devuelve el promedio raw.
 */
static esp_err_t ph_read_raw_average(int *raw_average)
{
    if (raw_average == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int raw_sum = 0;

    for (int i = 0; i < ADC_AVG_SAMPLES; i++) {
        int raw = 0;

        esp_err_t ret = adc_oneshot_read(
            adc_handle,
            PH_ADC_CHANNEL,
            &raw
        );

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error leyendo ADC: %s", esp_err_to_name(ret));
            return ret;
        }

        raw_sum += raw;
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_DELAY_MS));
    }

    *raw_average = raw_sum / ADC_AVG_SAMPLES;
    return ESP_OK;
}

void app_main(void)
{
    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, " Probador de modulo pH en GPIO34");
    ESP_LOGI(TAG, " Formula: pH = %.4f * V + %.4f", PH_SLOPE, PH_OFFSET);
    ESP_LOGI(TAG, " DIVIDER_RATIO = %.4f", DIVIDER_RATIO);
    ESP_LOGI(TAG, "======================================");

    esp_err_t ret = ph_adc_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo iniciar el probador de pH");
        return;
    }

    while (true) {
        int raw = 0;

        ret = ph_read_raw_average(&raw);
        if (ret == ESP_OK) {
            float voltage_gpio34 = adc_raw_to_voltage(raw);
            float voltage_po_estimated = voltage_gpio34 * DIVIDER_RATIO;

            /*
             * El pH se calcula con el voltaje que realmente entra al ADC.
             * Si estás usando divisor de voltaje, calibra PH_SLOPE y PH_OFFSET
             * usando voltage_gpio34, no voltage_po_estimated.
             */
            float ph = (PH_SLOPE * voltage_gpio34) + PH_OFFSET;

            printf("\n");
            printf("ADC raw promedio : %d\n", raw);
            printf("V_GPIO34         : %.3f V\n", voltage_gpio34);
            printf("V_PO_estimado    : %.3f V\n", voltage_po_estimated);
            printf("pH calculado     : %.2f\n", ph);

            if (voltage_gpio34 >= 3.20f) {
                printf("ADVERTENCIA      : V_GPIO34 esta muy cerca del limite de la ESP32.\n");
            }

            if (raw >= 4000) {
                printf("ADVERTENCIA      : ADC cerca de saturacion alta.\n");
            }

            if (raw <= 50) {
                printf("ADVERTENCIA      : ADC cerca de saturacion baja.\n");
            }

            if (ph < 0.0f || ph > 14.0f) {
                printf("ADVERTENCIA      : pH calculado fuera del rango 0-14.\n");
            }

            printf("--------------------------------------\n");
        }

        vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
    }
}