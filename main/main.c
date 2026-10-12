/*
 * Captura de CSI (Channel State Information) con ESP32-C5 conectado a un router Wi-Fi.
 *
 * Basado en espressif/esp-csi examples/get-started/csi_recv, adaptado para que la
 * fuente de tráfico sea un router doméstico (el ESP32-C5 es una estación STA normal).
 *
 * Versión 5 GHz (prueba). Flujo:
 *   1. Conexión STA al router, forzada a 5 GHz, 11a/11n (opcional 11ac), 20 MHz.
 *   2. Ping continuo al gateway (router) a CONFIG_CSI_PING_HZ: cada respuesta ICMP del router
 *      es una trama que llega al ESP32 y dispara el callback de CSI.
 *   3. El callback CSI copia los datos a una cola; una tarea aparte los imprime por serial
 *      como líneas CSV "CSI_DATA,..." para que un script de PC las guarde y grafique.
 *   4. Detector del bug esp-idf#18493 (en 5 GHz el buffer CSI sale siempre igual): cada
 *      CONFIG_CSI_BUGCHECK_WINDOW muestras imprime una línea "BUGCHECK" con cuántas
 *      muestras tenían el buffer idéntico a la anterior.
 *
 * Credenciales y opciones: idf.py menuconfig -> "CSI Router Config".
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "ping/ping_sock.h"

static const char *TAG = "csi_router";

#define CSI_QUEUE_LEN       64      /* muestras en espera de imprimirse */
#define CSI_LINE_MAX        8192    /* línea CSV máxima (holgura para cualquier longitud de buffer) */

/* Opciones bool de Kconfig como 0/1 (sin definir cuando están en "n") */
#ifdef CONFIG_CSI_5G_11AC
#define CSI_USE_11AC        1
#else
#define CSI_USE_11AC        0
#endif
#ifdef CONFIG_CSI_FORCE_LLTF
#define CSI_USE_LLTF        1
#else
#define CSI_USE_LLTF        0
#endif

/* Una muestra CSI copiada desde el callback (el buffer del driver solo es válido dentro del callback) */
typedef struct {
    uint32_t seq;               /* contador propio de muestras */
    uint32_t timestamp_us;      /* rx_ctrl.timestamp: tiempo local de recepción (us) */
    int8_t rssi;
    int8_t noise_floor;
    uint8_t rate;
    uint8_t channel;
    uint8_t second;             /* canal secundario (HT40) */
    uint8_t rx_format;          /* cur_bb_format: 11b/11g/HT/VHT/HE... */
    uint16_t sig_len;
    bool first_word_invalid;    /* los 4 primeros bytes del buffer no son válidos (limitación HW) */
    uint8_t mac[6];
    uint16_t len;               /* bytes de CSI (pares Im,Re int8 por subportadora) */
    int8_t buf[];
} csi_sample_t;

static QueueHandle_t s_csi_queue;
static uint8_t s_ap_bssid[6];
static volatile bool s_bssid_valid = false;
static volatile uint32_t s_dropped = 0;
static esp_ping_handle_t s_ping = NULL;

/* ------------------------------------------------------------------------- */
/* Callback CSI: se ejecuta en la tarea del driver Wi-Fi, por eso solo copia    */
/* los datos y los encola (nada de printf aquí: bloquearía el driver).          */
/* ------------------------------------------------------------------------- */
static void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *info)
{
    static uint32_t s_seq = 0;

    if (!info || !info->buf || info->len == 0) {
        return;
    }
    /* Solo nos interesan tramas transmitidas por NUESTRO router (su BSSID) */
    if (!s_bssid_valid || memcmp(info->mac, s_ap_bssid, 6) != 0) {
        return;
    }

    csi_sample_t *s = malloc(sizeof(csi_sample_t) + info->len);
    if (!s) {
        s_dropped++;
        return;
    }

    const wifi_pkt_rx_ctrl_t *rx = &info->rx_ctrl;
    s->seq = s_seq++;
    s->timestamp_us = rx->timestamp;
    s->rssi = rx->rssi;
    s->noise_floor = rx->noise_floor;
    s->rate = rx->rate;
    s->channel = rx->channel;
    s->second = rx->second;
    s->rx_format = rx->cur_bb_format;
    s->sig_len = rx->sig_len;
    s->first_word_invalid = info->first_word_invalid;
    memcpy(s->mac, info->mac, 6);
    s->len = info->len;                         /* longitud leída dinámicamente */
    memcpy(s->buf, info->buf, info->len);

    if (xQueueSend(s_csi_queue, &s, 0) != pdTRUE) {
        free(s);                                /* cola llena: se descarta la muestra */
        s_dropped++;
    }
}

/* ------------------------------------------------------------------------- */
/* Detector del bug esp-idf#18493: en 5 GHz el driver puede entregar siempre el */
/* mismo patrón IQ aunque RSSI y timestamp cambien. Compara cada muestra con la */
/* anterior y cada CONFIG_CSI_BUGCHECK_WINDOW muestras imprime un resumen.      */
/* ------------------------------------------------------------------------- */
static bool csi_same_buf(const csi_sample_t *a, const csi_sample_t *b)
{
    if (a->len != b->len) {
        return false;
    }
    /* Si la primera palabra es inválida (basura de HW), no se compara */
    int off = (a->first_word_invalid || b->first_word_invalid) ? 4 : 0;
    if (off >= a->len) {
        return false;
    }
    return memcmp(a->buf + off, b->buf + off, a->len - off) == 0;
}

static void csi_bugcheck(const csi_sample_t *prev, const csi_sample_t *s)
{
    static uint32_t n = 0, same = 0;
    static int8_t rssi_min = 127, rssi_max = -128;
    static bool warned_2g = false;

    if (!warned_2g && s->channel <= 14) {
        warned_2g = true;
        ESP_LOGW(TAG, "Canal %u: ¡NO estás en 5 GHz! Revisa el SSID en menuconfig", s->channel);
    }

    n++;
    if (prev && csi_same_buf(prev, s)) {
        same++;
    }
    if (s->rssi < rssi_min) rssi_min = s->rssi;
    if (s->rssi > rssi_max) rssi_max = s->rssi;

    if (n >= CONFIG_CSI_BUGCHECK_WINDOW) {
        ESP_LOGI(TAG, "BUGCHECK: buf identico %" PRIu32 "/%" PRIu32 " | RSSI %d..%d | canal %u | "
                 "rx_format %u | len %u", same, n, rssi_min, rssi_max, s->channel, s->rx_format, s->len);
        if (same * 10 >= n * 9) {
            ESP_LOGW(TAG, "BUGCHECK: >=90%% de buffers idénticos -> bug esp-idf#18493 presente, "
                     "los datos CSI no sirven (probar CSI_FORCE_LLTF)");
        }
        n = same = 0;
        rssi_min = 127;
        rssi_max = -128;
    }
}

/* ------------------------------------------------------------------------- */
/* Tarea de impresión: saca muestras de la cola y las escribe como CSV.         */
/* ------------------------------------------------------------------------- */
static void csi_print_task(void *arg)
{
    static char line[CSI_LINE_MAX];
    uint32_t last_dropped = 0;
    csi_sample_t *prev = NULL;                  /* muestra anterior, para el detector del bug */

    printf("type,seq,mac,rssi,rate,noise_floor,channel,secondary_channel,local_timestamp,"
           "sig_len,rx_format,len,first_word_invalid,data\n");

    while (1) {
        csi_sample_t *s;
        if (xQueueReceive(s_csi_queue, &s, pdMS_TO_TICKS(5000)) != pdTRUE) {
            ESP_LOGW(TAG, "Sin CSI en 5 s (¿conectado? ¿el router responde al ping?)");
            continue;
        }

#if CSI_USE_LLTF
        /* L-LTF forzado: valores de 12 bits empaquetados en 2 bytes (little endian) */
        uint16_t nvals = s->len / 2;
#else
        uint16_t nvals = s->len;
#endif
        /* "len" = número de valores impresos, para que coincida con el contenido de data */
        int n = snprintf(line, sizeof(line),
                         "CSI_DATA,%" PRIu32 "," MACSTR ",%d,%u,%d,%u,%u,%" PRIu32 ",%u,%u,%u,%d,\"[",
                         s->seq, MAC2STR(s->mac), s->rssi, s->rate, s->noise_floor,
                         s->channel, s->second, s->timestamp_us, s->sig_len,
                         s->rx_format, nvals, s->first_word_invalid);

        /* Valores alternados [Im0,Re0,Im1,Re1,...] */
        for (int i = 0; i < nvals && n < (int)sizeof(line) - 8; i++) {
#if CSI_USE_LLTF
            const uint8_t *b = (const uint8_t *)s->buf;
            int16_t v = ((int16_t)(((b[2 * i + 1] << 8) | b[2 * i]) << 4)) >> 4;
#else
            int v = s->buf[i];
#endif
            n += snprintf(line + n, sizeof(line) - n, i ? ",%d" : "%d", v);
        }
        n += snprintf(line + n, sizeof(line) - n, "]\"\n");
        fwrite(line, 1, n, stdout);

        csi_bugcheck(prev, s);
        free(prev);
        prev = s;

        if (s_dropped != last_dropped) {
            last_dropped = s_dropped;
            ESP_LOGW(TAG, "Muestras descartadas acumuladas: %" PRIu32, last_dropped);
        }
    }
}

/* ------------------------------------------------------------------------- */
/* Generador de tráfico: ping continuo al router. Sin esto, un STA inactivo     */
/* recibe muy pocas tramas OFDM y el CSI sería esporádico.                      */
/* ------------------------------------------------------------------------- */
static void ping_router_start(const esp_ip4_addr_t *gw)
{
    if (s_ping) {                               /* reconexión: reusar la sesión */
        esp_ping_start(s_ping);
        return;
    }

    esp_ping_config_t cfg = ESP_PING_DEFAULT_CONFIG();
    cfg.count = ESP_PING_COUNT_INFINITE;
    cfg.interval_ms = 1000 / CONFIG_CSI_PING_HZ;
    cfg.timeout_ms = 500;
    cfg.data_size = 1;
    cfg.task_stack_size = 3072;
    cfg.target_addr.type = ESP_IPADDR_TYPE_V4;
    cfg.target_addr.u_addr.ip4.addr = gw->addr;

    esp_ping_callbacks_t cbs = { 0 };
    ESP_ERROR_CHECK(esp_ping_new_session(&cfg, &cbs, &s_ping));
    ESP_ERROR_CHECK(esp_ping_start(s_ping));
    ESP_LOGI(TAG, "Ping al router " IPSTR " cada %" PRIu32 " ms", IP2STR(gw), cfg.interval_ms);
}

/* ------------------------------------------------------------------------- */
/* Eventos Wi-Fi/IP: guarda el BSSID del router, arranca el ping y reconecta.   */
/* ------------------------------------------------------------------------- */
static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_CONNECTED) {
        wifi_event_sta_connected_t *ev = data;
        memcpy(s_ap_bssid, ev->bssid, 6);
        s_bssid_valid = true;
        ESP_LOGI(TAG, "Conectado a \"%s\" BSSID " MACSTR " canal %d",
                 (char *)ev->ssid, MAC2STR(ev->bssid), ev->channel);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *ev = data;
        ESP_LOGW(TAG, "Desconectado (reason %d), reintentando...", ev->reason);
        if (s_ping) {
            esp_ping_stop(s_ping);
        }
        esp_wifi_connect();                     /* reconexión básica */
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = data;
        ESP_LOGI(TAG, "IP " IPSTR ", gateway " IPSTR, IP2STR(&ev->ip_info.ip), IP2STR(&ev->ip_info.gw));
        ping_router_start(&ev->ip_info.gw);
    }
}

/* ------------------------------------------------------------------------- */
/* Inicialización STA forzada a 5 GHz, 802.11a/n (opcional ac), 20 MHz.         */
/* ------------------------------------------------------------------------- */
static void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_CSI_WIFI_SSID,
            .password = CONFIG_CSI_WIFI_PASSWORD,
            .scan_method = WIFI_ALL_CHANNEL_SCAN,
            .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
            .threshold.authmode = WIFI_AUTH_OPEN,   /* acepta abierta/WPA2/WPA3 */
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* Solo 5 GHz: aunque el router sea dual-band, el STA no se asocia en 2.4 GHz */
    ESP_ERROR_CHECK(esp_wifi_set_band_mode(WIFI_BAND_MODE_5G_ONLY));

    /* Sin 11ax: el router responde con tramas HT (o VHT si se habilita 11ac). 20 MHz:
       mismo nº de subportadoras que la prueba de 2.4 GHz, para comparar en igualdad.
       Se rellena también .ghz_2g (igual que la versión 2.4 GHz) para que la config sea válida. */
    wifi_protocols_t protocols = {
        .ghz_2g = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N,
        .ghz_5g = WIFI_PROTOCOL_11A | WIFI_PROTOCOL_11N
#if CSI_USE_11AC
                | WIFI_PROTOCOL_11AC
#endif
        ,
    };
    ESP_ERROR_CHECK(esp_wifi_set_protocols(WIFI_IF_STA, &protocols));
    wifi_bandwidths_t bw = { .ghz_2g = WIFI_BW20, .ghz_5g = WIFI_BW20 };
    ESP_ERROR_CHECK(esp_wifi_set_bandwidths(WIFI_IF_STA, &bw));

    /* Sin ahorro de energía: la radio siempre escucha, no se pierden tramas */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    ESP_LOGI(TAG, "Conectando a \"%s\" (solo 5 GHz, 11a/n%s, 20 MHz, force_lltf=%d)...",
             CONFIG_CSI_WIFI_SSID, CSI_USE_11AC ? "/ac" : "", CSI_USE_LLTF);
    ESP_ERROR_CHECK(esp_wifi_connect());
}

/* ------------------------------------------------------------------------- */
/* Configuración CSI (ESP32-C5): qué tipo de preámbulo se usa para estimar el   */
/* canal. Con HT-LTF (802.11n) se obtienen las subportadoras de datos.          */
/* ------------------------------------------------------------------------- */
static void wifi_csi_init(void)
{
    wifi_csi_config_t csi_config = {
        .enable                   = true,
        .acquire_csi_legacy       = false,  /* L-LTF de tramas 11a/g: desactivado */
        .acquire_csi_force_lltf   = CSI_USE_LLTF, /* 1: datos en 12 bits (se desempaquetan al imprimir) */
        .acquire_csi_ht20         = true,   /* HT-LTF en tramas 802.11n de 20 MHz */
        .acquire_csi_ht40         = true,   /* HT-LTF en tramas 802.11n de 40 MHz */
        .acquire_csi_vht          = CSI_USE_11AC, /* VHT-LTF si el router responde en 11ac */
        .acquire_csi_su           = false,  /* HE (11ax) desactivado en el STA */
        .acquire_csi_mu           = false,
        .acquire_csi_dcm          = false,
        .acquire_csi_beamformed   = false,
        .acquire_csi_he_stbc_mode = 2,
        .val_scale_cfg            = 0,
        .dump_ack_en              = false,
    };
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_config));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(wifi_csi_rx_cb, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    s_csi_queue = xQueueCreate(CSI_QUEUE_LEN, sizeof(csi_sample_t *));
    xTaskCreate(csi_print_task, "csi_print", 4096, NULL, 5, NULL);

    wifi_init_sta();
    wifi_csi_init();
    /* app_main termina; el driver Wi-Fi, el ping y csi_print siguen corriendo indefinidamente */
}
