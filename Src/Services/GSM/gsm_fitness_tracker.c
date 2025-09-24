#include "COMMON_CONFIG.h"
#include "gsm_fitness_tracker.h"
#include "uart_fitness_tracker.h"
#define _GNU_SOURCE
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include <string.h>

#include <string.h>
#include <ctype.h>
#include <stdbool.h>
#include <stdio.h>

/* --------------------------------------------------------------------------
 * Goal: make this file self-sufficient for M95 bring-up while reusing the
 * UART layer from uart_fitness_tracker.c. Adds:
 *  - Open-drain PWRKEY pulses with correct timings
 *  - Autobaud sync and baud lock (ATE0, IFC, IPR, &W)
 *  - Send/expect helpers with token scanning + timeouts
 *  - Command table that includes expected replies + timeouts
 *  - Safer GSM_PowerOn/Off and a minimal init path
 * -------------------------------------------------------------------------- */

/* ===================== Token scanner state ===================== */
#define GSM_SCAN_BUF_SZ 512
static char s_scan_buf[GSM_SCAN_BUF_SZ];
static volatile size_t s_scan_len = 0; // bytes in scan buffer
static volatile bool s_flag_ok = false;
static volatile bool s_flag_error = false;
static volatile bool s_flag_prompt = false; // '>' prompt (e.g., CMGS or QISEND)
static volatile bool s_flag_rdy = false;    // 'RDY' seen
volatile bool response_received = false;    // legacy flag retained (not primary)

static gsm_cmd_t gsm_cmd_tracker;
static uint16_t last_cmd_index = 0;

static const gsm_cmd_t gsm_commands[CMD_COUNT] = {
    [CMD_AT] = {"AT\r", "OK", 800},
    [CMD_ATE0] = {"ATE0\r", "OK", 800},
    [CMD_ATIFC] = {"AT+IFC=0,0\r", "OK", 1200},
    [CMD_AT_IPR] = {"AT+IPR=115200\r", "OK", 1500},
    [CMD_AT_COPS_SCAN] = {"AT+COPS=?\r", "OK", 60000},
    [CMD_AT_COPS_PLMN] = {"AT+COPS=0\r", "OK", 15000},
    [CMD_AT_COPS_Q] = {"AT+COPS?\r", "+COPS:", 1500},
    [CMD_AT_COPS_MANUAL] = {"AT+COPS=1,2,\"%d\"\r", "OK", 20000}, // e.g. "23415" for Vodafone UK
    // Save settings
    [CMD_AT_W] = {"AT&W\r", "OK", 1200},
    [CMD_ATI] = {"ATI\r", "OK", 1200},
    [CMD_AT_GSN] = {"AT+GSN\r", "OK", 1200},
    [CMD_AT_CPIN] = {"AT+CPIN?\r", "+CPIN: READY", 1500},
    [CMD_AT_CREG_Q] = {"AT+CREG?\r", "+CREG:", 1500},
    [CMD_AT_CREG_NETWORK_REGISTER] = {"AT+CREG=%d\r", "OK", 800},
    [CMD_AT_CFUN_REBOOT] = {"AT+CFUN=1,1\r", "OK", 15000},
    [CMD_AT_CMEE] = {"AT+CMEE=2\r", "OK", 800},
    [CMD_AT_CSQ] = {"AT+CSQ\r", "+CSQ:", 1500},
    [CMD_AT_QPING] = {"AT+QPING=1,\"%s\"", "OK", 10000},
    [CMD_AT_CGATT] = {"AT+CGATT=1\r", "OK", 10000},
    [CMD_AT_CGATT_Q] = {"AT+CGATT?\r", "+CGATT:", 1500},
    [CMD_AT_QBAND_VALUE] = {"AT+QBAND=%d\r", "+QBAND:", 1500},
    // PDP setup
    [CMD_AT_QIFGCNT] = {"AT+QIFGCNT=0\r", "OK", 800},
    [CMD_AT_QICSGP] = {"AT+QICSGP=1,%s\r", "OK", 1500},
    [CMD_AT_QICSGP_Q] = {"AT+QICSGP?\r", "+QICSGP:", 800},
    [CMD_AT_QIREGAPP_Q] = {"AT+QIREGAPP?\r", "+QIREGAPP:", 800},
    [CMD_AT_CGREG_Q] = {"AT+CGREG?\r", "+CGREG:", 1500},
    [CMD_AT_QIACT_Q] = {"AT+QIACT?\r", "+QIACT:", 1500},
    [CMD_AT_CGPADDR] = {"AT+CGPADDR\r", "+CGPADDR:", 2000},
    // GPRS data connection
    [CMD_AT_QIMUX] = {"AT+QIMUX=0\r", "OK", 800},
    [CMD_AT_QIMODE] = {"AT+QIMODE=0\r", "OK", 800},
    [CMD_AT_QINDI] = {"AT+QINDI=1\r", "OK", 800},
    [CMD_AT_QIDNSIP] = {"AT+QIDNSIP=1\r", "OK", 800},
    [CMD_AT_QIREGAPP] = {"AT+QIREGAPP\r", "OK", 1500},
    [CMD_AT_QIACT] = {"AT+QIACT\r", "OK", 20000},
    [CMD_AT_QILOCIP] = {"AT+QILOCIP\r", ".", 2000},

    // DNS + TCP connect to broker
    [CMD_AT_QIDNSGIP] = {"AT+QIDNSGIP=\"broker.example.com\"\r", "OK", 15000},
    [CMD_AT_QMTOPEN] = {"AT+QMTOPEN=0,\"%s\",1883\r", "OK", 30000},
    [CMD_AT_QMTOPEN_Q] = {"AT+QMTOPEN?\r", "+QMTOPEN:", 800},

    // tune send prompts/echo
    [CMD_AT_QIPROMPT] = {"AT+QIPROMPT=1\r", "OK", 800},
    [CMD_AT_QISDE] = {"AT+QISDE=0\r", "OK", 800},

    // MQTT CONNECT: first wait for '>' then for 'SEND OK'
    [CMD_AT_QISEND_MQTT_CONNECT] = {"AT+QISEND=30\r", ">", 2000},
    [CMD_AT_QMTCONN] = {
        "AT+QMTCONN=0,\"Client\"",
        "OK",
        1000,
    }, //,\"user\",\"pass\"\r", "OK", 10000 },
    [CMD_AT_QMTCONN_Q] = {"AT+QMTCONN=?\r", "+QMTCONN:", 800},
    [CMD_MQTT_CONNECT_PAYLOAD] = {"\x10\x1E\x00\x04MQTT\x04\xC2\x00\x3C\x00\x06fit-01\x00\x04user\x00\x04pass\x1A", "SEND OK", 5000},

    // SUBSCRIBE QoS0 to tracker/cmd
    [CMD_AT_QMTSUB] = {"AT+QMTSUB=0,1,\"tracker/cmd\",0\r", "OK", 10000},
    // PUBLISH QoS0 topic tracker/data, payload "hello"
    [CMD_AT_QMTPUB] = {"AT+QMTPUB=0,0,0,0,\"tracker/data\"\r", ">", 2000},
    // Keepalive (PINGREQ)
    [CMD_AT_QISEND_KEEPALIVE] = {"AT+QISEND=2\r", ">", 2000},
    [CMD_MQTT_KEEPALIVE_PAYLOAD] = {"\xC0\x00\x1A", "SEND OK", 2000},

    // Disconnect
    [CMD_AT_QMTDISC] = {"AT+QMTDISC=0\r", "OK", 10000},

    // Clean down
    [CMD_AT_QICLOSE] = {"AT+QICLOSE\r", "CLOSE OK", 5000},
    [CMD_AT_QIDEACT] = {"AT+QIDEACT\r", "DEACT OK", 10000},
    [CMD_AT_QPOWD] = {"AT+QPOWD=1\r", "NORMAL POWER DOWN", 12000},
};

static inline void gsm_flags_clear(void)
{
    s_flag_ok = s_flag_error = s_flag_prompt = s_flag_rdy = false;
}

static bool parse_model_revision_simple(const char *buf,
                                        char *model, size_t model_len,
                                        char *rev, size_t rev_len)
{
    if (!buf || !model || !rev || model_len == 0 || rev_len == 0)
        return false;

    model[0] = rev[0] = '\0';

    // Make a modifiable copy for strtok_r
    size_t n = strlen(buf);
    char *tmp = (char *)malloc(n + 1);
    if (!tmp)
        return false;
    memcpy(tmp, buf, n + 1);

    char *save = NULL;
    for (char *line = strtok_r(tmp, "\r\n", &save);
         line;
         line = strtok_r(NULL, "\r\n", &save))
    {
        // left trim
        while (*line && isspace((unsigned char)*line))
            line++;

        if (!*line)
            continue;

        // Model: first "Quectel_*" but not "Quectel_Ltd"
        if (model[0] == '\0' &&
            strncmp(line, "Quectel_", 8) == 0 &&
            strcmp(line, "Quectel_Ltd") != 0)
        {
            strncpy(model, line, model_len - 1);
            model[model_len - 1] = '\0';
        }

        // Revision: "Revision: <value>"
        if (rev[0] == '\0' && strncmp(line, "Revision:", 9) == 0)
        {
            char *p = line + 9;
            while (*p && isspace((unsigned char)*p))
                p++;
            if (*p)
            {
                strncpy(rev, p, rev_len - 1);
                rev[rev_len - 1] = '\0';
            }
        }

        if (model[0] && rev[0])
            break; // done early
    }

    free(tmp);
    return (model[0] && rev[0]);
}

static bool powered_on(const char *response)
{
    if (strstr(response, "NORMAL POWER DOWN") != NULL)
    {
        return false; // Module is powered off
    }
    else if (strstr(response, "RDY") != NULL)
    {
        return true; // Module is powered on
    }
    return true;
}

static void gsm_scan_chunk(const uint8_t *p, size_t n)
{
    switch (last_cmd_index)
    {
    case CMD_POWER_ON:
        if (!powered_on((const char *)p))
        {
            device_info.gsm.is_powered_on = false;
            NRF_LOG_INFO("GSM Module is powered off");
            NRF_LOG_FLUSH();
        }
        else
        {
            device_info.gsm.is_powered_on = true;
        }
        break;

    case CMD_AT:
    case CMD_ATE0:
    case CMD_ATIFC:
    case CMD_AT_IPR:
    case CMD_AT_W:
    case CMD_AT_CMEE:
    case CMD_AT_CREG_NETWORK_REGISTER:
    case CMD_AT_CGATT:
    case CMD_AT_QIFGCNT:
    case CMD_AT_QICSGP:
    case CMD_AT_QIMUX:
    case CMD_AT_QIMODE:
    case CMD_AT_QINDI:
    case CMD_AT_QIDNSIP:
    case CMD_AT_QIREGAPP:
    case CMD_AT_QIACT:
        if (strstr((char *)p, gsm_cmd_tracker.expect) != NULL)
        {
            s_flag_ok = true;
        }
        else
        {
            s_flag_error = true;
        }
        break;

    case CMD_ATI:
        s_flag_prompt = parse_model_revision_simple((const char *)p, device_info.gsm.model, sizeof(device_info.gsm.model), device_info.gsm.Revision, sizeof(device_info.gsm.Revision));
        break;
    case CMD_AT_QILOCIP:
        if (strstr((char *)p, gsm_cmd_tracker.expect) != NULL)
        {
            char *ip_pos = strstr((char *)p, "+QILOCIP:");
            if (ip_pos)
            {
                char ip[32] = {0};
                if (sscanf(ip_pos, "+QILOCIP: \"%31[^\"]\"", ip) == 1)
                {
                    strncpy(device_info.gsm.ip_address, ip, sizeof(device_info.gsm.ip_address) - 1);
                    device_info.gsm.ip_address[sizeof(device_info.gsm.ip_address) - 1] = '\0';
                    s_flag_ok = true;
                }
            }
        }
        break;
    case CMD_AT_GSN:
        // s_flag_prompt = parse_u32_fast((const char*)p, (uint32_t*)device_info.gsm.imei);
        device_info.gsm.imei = strtoll(p, NULL, 10);
        if (device_info.gsm.imei > 0)
            s_flag_prompt = true;
        break;
    case CMD_AT_CPIN:
        if (strstr((char *)p, gsm_cmd_tracker.expect) != NULL)
        {
            device_info.gsm.sim_ready = true;
            s_flag_ok = true;
        }
        else
        {
            NRF_LOG_ERROR("SIM not ready");
            device_info.gsm.sim_ready = false;
        }
        break;

    case CMD_AT_CREG_Q:
        if (strstr((char *)p, gsm_cmd_tracker.expect) != NULL)
        {
            // Example response: +CREG: 0,1 or +CREG: 0,5
            char *creg_pos = strstr((char *)p, "+CREG:");
            if (creg_pos)
            {
                int n, stat;
                if (sscanf(creg_pos, "+CREG: %d,%d", &n, &stat) == 2)
                {
                    if (stat == 1 || stat == 5)
                    { // Registered
                        device_info.gsm.network_registered = true;
                        s_flag_ok = true;
                    }
                    else
                    {
                        device_info.gsm.network_registered = false;
                        s_flag_error = true;
                    }
                }
            }
        }
        break;

    case CMD_AT_CSQ:
        if (strstr((char *)p, gsm_cmd_tracker.expect) != NULL)
        {
            // Example response: +CSQ: 15,99
            char *csq_pos = strstr((char *)p, "+CSQ:");
            if (csq_pos)
            {
                int rssi, ber;
                if (sscanf(csq_pos, "+CSQ: %d,%d", &rssi, &ber) == 2)
                {
                    if (rssi != 99)
                    { // 99 means not known or not detectable
                        s_flag_ok = true;
                    }
                    else
                    {
                        s_flag_error = true;
                    }
                }
            }
        }
        break;
    case CMD_AT_QMTOPEN:
        if (strstr((char *)p, gsm_cmd_tracker.expect) != NULL)
        {
            // Example response: +QMTOPEN: 0,0
            char *qmtopen_pos = strstr((char *)p, "+QMTOPEN:");
            if (qmtopen_pos)
            {
                int client_id, err_code;
                if (sscanf(qmtopen_pos, "+QMTOPEN: %d,%d", &client_id, &err_code) == 2)
                {
                    if (err_code == 0)
                    { // Success
                        s_flag_ok = true;
                    }
                    else
                    {
                        s_flag_error = true;
                    }
                }
            }
        }
        break;
    case CMD_AT_QMTOPEN_Q:
        if (strstr((char *)p, gsm_cmd_tracker.expect) != NULL)
        {
            // Example response: +QMTOPEN: 0,0,1883,"broker.example.com"
            char *qmtopen_pos = strstr((char *)p, "+QMTOPEN:");
            if (qmtopen_pos)
            {
                int client_id, port;
                char server[64];
                if (sscanf(qmtopen_pos, "+QMTOPEN: %d,%s,%d", &client_id, server, &port) >= 3)
                {
                    if (port > 0)
                    { // Success
                        s_flag_ok = true;
                    }
                    else
                    {
                        s_flag_error = true;
                    }
                }
            }
        }
        break;
    case CMD_AT_QMTCONN:
        if (strstr((char *)p, gsm_cmd_tracker.expect) != NULL)
        {
            // Example response: +QMTCONN: 0,0
            char *qmtconn_pos = strstr((char *)p, "+QMTCONN:");
            if (qmtconn_pos)
            {
                int client_id, err_code;
                if (sscanf(qmtconn_pos, "+QMTCONN: %d,%d", &client_id, &err_code) == 2)
                {
                    if (err_code == 0)
                    { // Success
                        device_info.gsm.mqtt_connected = true;
                        s_flag_ok = true;
                    }
                    else
                    {
                        device_info.gsm.mqtt_connected = false;
                        s_flag_error = true;
                    }
                }
            }
        }
        break;
    case CMD_AT_QMTCONN_Q:
        if (strstr((char *)p, gsm_cmd_tracker.expect) != NULL)
        {
            // Example response: +QMTCONN: 0,0
            char *qmtconn_pos = strstr((char *)p, "+QMTCONN:");
            if (qmtconn_pos)
            {
                int client_id;
                char *client;
                if (sscanf(qmtconn_pos, "+QMTCONN: %d,%s", &client_id, &client) == 2)
                {
                    if (strlen(client) > 0)
                    { // Connected
                        device_info.gsm.mqtt_connected = true;
                        s_flag_ok = true;
                    }
                    else
                    {
                        device_info.gsm.mqtt_connected = false;
                        s_flag_error = true;
                    }
                }
            }
        }
        break;
    case CMD_AT_QMTSUB:
        if (strstr((char *)p, gsm_cmd_tracker.expect) != NULL)
        {
            // Example response: +QMTSUB: 0,1,0,0
            char *qmtsub_pos = strstr((char *)p, "+QMTSUB:");
            if (qmtsub_pos)
            {
                int client_id, msg_id, qos, err_code;
                if (sscanf(qmtsub_pos, "+QMTSUB: %d,%d,%d,%d", &client_id, &msg_id, &qos, &err_code) == 4)
                {
                    if (err_code == 0)
                    { // Success
                        s_flag_ok = true;
                    }
                    else
                    {
                        s_flag_error = true;
                    }
                }
            }
        }
        break;

    case CMD_AT_QMTDISC:
        if (strstr((char *)p, gsm_cmd_tracker.expect) != NULL)
        {
            // Example response: +QMTDISC: 0,0
            char *qmtdisc_pos = strstr((char *)p, "+QMTDISC:");
            if (qmtdisc_pos)
            {
                int client_id, err_code;
                if (sscanf(qmtdisc_pos, "+QMTDISC: %d,%d", &client_id, &err_code) == 2)
                {
                    if (err_code == 0)
                    { // Success
                        device_info.gsm.mqtt_connected = false;
                        s_flag_ok = true;
                    }
                    else
                    {
                        s_flag_error = true;
                    }
                }
            }
        }
        break;
    case CMD_AT_CGATT_Q:
        if (strstr((char *)p, gsm_cmd_tracker.expect) != NULL)
        {
            // Example response: +CGATT: 1
            char *cgatt_pos = strstr((char *)p, "+CGATT:");
            if (cgatt_pos)
            {
                int status;
                if (sscanf(cgatt_pos, "+CGATT: %d", &status) == 1)
                {
                    if (status == 1)
                    { // Attached
                        device_info.gsm.gprs_attached = true;
                        s_flag_ok = true;
                    }
                    else
                    {
                        device_info.gsm.gprs_attached = false;
                        s_flag_error = true;
                    }
                }
            }
        }
        break;

    case GSM_WAIT_CREG:
        if (strstr((char *)p, "+CREG:") != NULL)
        {
            // Example response: +CREG: 0,1 or +CREG: 0,5
            char *creg_pos = strstr((char *)p, "+CREG:");
            if (creg_pos)
            {
                uint8_t n;
                char lac[16] = {0}, ci[16] = {0};

                int parsed = sscanf(creg_pos, "+CREG: %d,\"%15[^\"]\",\"%15[^\"]",
                                    &n, lac, ci);
                if (parsed >= 1)
                {
                    NRF_LOG_INFO("n=%d", n);
                    if (n == 1 || n == 5)
                    { // Registered
                        device_info.gsm.network_registered = true;
                        s_flag_ok = true;
                    }
                    else
                    {
                        device_info.gsm.network_registered = false;
                        s_flag_error = true;
                    }
                }
            }
        }

        break;

    default:
        break;
    }
}

/* ===================== UART callback bridge ===================== */

void process_gsm_response(char *response, uint8_t length)
{
    if (!response || length <= 1)
        return;

    // Log trimmed chunk
    NRF_LOG_INFO("GSM Response: %s", response);
    NRF_LOG_FLUSH();

    gsm_scan_chunk((const uint8_t *)response, length);

    response_received = true; // legacy; some older code may still gate on this
}

/* ===================== Low-level TX helper ===================== */
static bool gsm_try_tx(const uint8_t *p, size_t n, uint32_t timeout_ms)
{

    uint32_t waited = 0;
    while (uarte_transmit(GSM_UART, p, n) != NRFX_SUCCESS)
    {
        if (waited >= timeout_ms)
            return false;
        nrf_delay_ms(5);
        waited += 5;
    }
    return true;
}

/* ===================== Send/expect with timeout ===================== */
static bool gsm_send_expect(const char *cmd, const char *expect, uint32_t timeout_ms)
{
    gsm_flags_clear();
    if (cmd)
    {
        size_t n = strlen(cmd);
        if (!gsm_try_tx((const uint8_t *)cmd, n, 500))
        {
            NRF_LOG_ERROR("GSM TX busy for cmd: %s", cmd);
            return false;
        }
    }
    response_received = false; // reset legacy flag
    while (!response_received)
    {
    }

    if (s_flag_ok)
    {
        return true;
    }
    else if (s_flag_prompt)
    {
        NRF_LOG_INFO("Prompt received");
        return true;
    }
    else if (s_flag_error)
    {
        return false;
    }
    else
    {
        return false;
    }

    return false;
}

/* ===================== PWRKEY helpers (M95) ===================== */
static void pwrkey_pulse_ms(uint32_t ms)
{
    // Emulate open-drain: drive LOW, then release (Hi-Z)
    nrf_gpio_cfg_output(PIN_GSM_PWRKEY);
    nrf_gpio_pin_set(PIN_GSM_PWRKEY); // assert LOW
    nrf_delay_ms(ms);
    nrf_gpio_cfg_input(PIN_GSM_PWRKEY, NRF_GPIO_PIN_NOPULL); // release
}

static bool m95_power_on(void)
{
    uint8_t count = 0;
    device_info.gsm.is_powered_on = true;
    last_cmd_index = CMD_POWER_ON;
    nrf_delay_ms(100);     // VBAT settle (handled by board power-up)
    pwrkey_pulse_ms(1200); // ON requires >1 s LOW
    while (count < 50)
    {
        count++;
        if (device_info.gsm.is_powered_on != true)
        {
            NRF_LOG_INFO("power reset the nrf board");
            NRF_LOG_FLUSH();
            return false;
        }
        nrf_delay_ms(100);
    }
    return true;
}

static void m95_power_off(void)
{
    pwrkey_pulse_ms(850); // OFF requires ~0.7–1.0 s LOW
}

/* ===================== Public-ish API kept compatible ===================== */
static bool GSM_SendCommand_str(uint16_t cmd_index, char *str)
{
    char tempBuffer[128];
    snprintf(tempBuffer, sizeof(tempBuffer), gsm_commands[cmd_index].cmd, str);
    if (cmd_index >= CMD_COUNT || device_info.gsm.is_powered_on == false)
        return false;
    gsm_cmd_tracker.cmd = tempBuffer;
    gsm_cmd_tracker.expect = gsm_commands[cmd_index].expect;
    gsm_cmd_tracker.timeout_ms = gsm_commands[cmd_index].timeout_ms;
    last_cmd_index = cmd_index;
    NRF_LOG_INFO("GSM Cmd[%u]", (unsigned)cmd_index);
    NRF_LOG_INFO("cmd %s", nrf_log_push(gsm_cmd_tracker.cmd ? gsm_cmd_tracker.cmd : "<any>"));
    NRF_LOG_INFO("expect: %s", nrf_log_push(gsm_cmd_tracker.expect ? gsm_cmd_tracker.expect : "<any>"));
    NRF_LOG_INFO("timeout: %u ms", (unsigned)gsm_cmd_tracker.timeout_ms);
    NRF_LOG_FLUSH();
    bool ok = gsm_send_expect(gsm_cmd_tracker.cmd, gsm_cmd_tracker.expect, gsm_cmd_tracker.timeout_ms);
    if (!ok)
    {
        NRF_LOG_WARNING("Timeout/No match for expect: %s", gsm_cmd_tracker.expect ? gsm_cmd_tracker.expect : "<any>");
    }
    return ok;
}
static bool GSM_SendCommand_uint(uint16_t cmd_index, uint16_t val)
{
    char tempBuffer[32];
    snprintf(tempBuffer, sizeof(tempBuffer), gsm_commands[cmd_index].cmd, val);
    if (cmd_index >= CMD_COUNT || device_info.gsm.is_powered_on == false)
        return false;
    gsm_cmd_tracker.cmd = tempBuffer;
    gsm_cmd_tracker.expect = gsm_commands[cmd_index].expect;
    gsm_cmd_tracker.timeout_ms = gsm_commands[cmd_index].timeout_ms;
    last_cmd_index = cmd_index;
    NRF_LOG_INFO("GSM Cmd[%u]", (unsigned)cmd_index);
    NRF_LOG_INFO("cmd %s", nrf_log_push(gsm_cmd_tracker.cmd ? gsm_cmd_tracker.cmd : "<any>"));
    NRF_LOG_INFO("expect: %s", nrf_log_push(gsm_cmd_tracker.expect ? gsm_cmd_tracker.expect : "<any>"));
    NRF_LOG_INFO("timeout: %u ms", (unsigned)gsm_cmd_tracker.timeout_ms);
    NRF_LOG_FLUSH();
    bool ok = gsm_send_expect(gsm_cmd_tracker.cmd, gsm_cmd_tracker.expect, gsm_cmd_tracker.timeout_ms);
    if (!ok)
    {
        NRF_LOG_WARNING("Timeout/No match for expect: %s", gsm_cmd_tracker.expect ? gsm_cmd_tracker.expect : "<any>");
    }
    return ok;
}
bool GSM_SendCommand(uint16_t cmd_index)
{
    if (cmd_index >= CMD_COUNT || device_info.gsm.is_powered_on == false)
        return false;
    // strcpy((char*)&gsm_cmd_tracker, (const char*)&gsm_commands[cmd_index]);
    // strcpy((char*)gsm_cmd_tracker.expect, (const char*)gsm_commands[cmd_index].expect);
    gsm_cmd_tracker.cmd = gsm_commands[cmd_index].cmd;
    gsm_cmd_tracker.expect = gsm_commands[cmd_index].expect;
    gsm_cmd_tracker.timeout_ms = gsm_commands[cmd_index].timeout_ms;
    last_cmd_index = cmd_index;
    NRF_LOG_INFO("GSM Cmd[%u]", (unsigned)cmd_index);
    NRF_LOG_INFO("cmd %s", nrf_log_push(gsm_cmd_tracker.cmd ? gsm_cmd_tracker.cmd : "<any>"));
    NRF_LOG_INFO("expect: %s", nrf_log_push(gsm_cmd_tracker.expect ? gsm_cmd_tracker.expect : "<any>"));
    NRF_LOG_INFO("timeout: %u ms", (unsigned)gsm_cmd_tracker.timeout_ms);
    NRF_LOG_FLUSH();
    bool ok = gsm_send_expect(gsm_cmd_tracker.cmd, gsm_cmd_tracker.expect, gsm_cmd_tracker.timeout_ms);
    if (!ok)
    {
        NRF_LOG_WARNING("Timeout/No match for expect: %s", gsm_cmd_tracker.expect ? gsm_cmd_tracker.expect : "<any>");
    }
    return ok;
}

/* ===================== Minimal bring-up / checks ===================== */
static bool m95_autobaud_and_lock(void)
{
    // Send AT a few times until we get OK (autobaud)
    for (int i = 0; i < 20; ++i)
    {

        if (GSM_SendCommand(CMD_AT))
        {
            NRF_LOG_INFO("M95: autobaud synced");
            NRF_LOG_FLUSH();
            // Disable echo, HW flow control (unless wired), set fixed baud and save
            GSM_SendCommand(CMD_ATE0);
            GSM_SendCommand(CMD_ATIFC);
            GSM_SendCommand(CMD_AT_IPR);
            GSM_SendCommand(CMD_AT_W);
            return true;
        }
        nrf_delay_ms(250);
    }
    NRF_LOG_WARNING("M95: autobaud sync failed");
    return false;
}

static bool m95_check_ready(uint16_t cmd_index)
{
    gsm_cmd_tracker.cmd = gsm_commands[cmd_index].cmd;
    gsm_cmd_tracker.expect = gsm_commands[cmd_index].expect;
    gsm_cmd_tracker.timeout_ms = gsm_commands[cmd_index].timeout_ms;
    last_cmd_index = cmd_index;

    gsm_flags_clear();
    if (gsm_cmd_tracker.cmd)
    {
        size_t n = strlen(gsm_cmd_tracker.cmd);
        if (!gsm_try_tx((const uint8_t *)gsm_cmd_tracker.cmd, n, 500))
        {
            NRF_LOG_ERROR("GSM TX busy for cmd: %s", gsm_cmd_tracker.cmd);
            return false;
        }
    }
    response_received = false; // reset legacy flag
    uint8_t delay_count = 0;
    while (!response_received && delay_count < (gsm_cmd_tracker.timeout_ms / 10))
    {
        nrf_delay_ms(10);
        delay_count++;
    }

    if (s_flag_ok)
    {
        return true;
    }
    else if (s_flag_prompt)
    {
        NRF_LOG_INFO("Prompt received");
        return true;
    }
    else if (s_flag_error)
    {
        return false;
    }
    else
    {
        return false;
    }
}

/* ===================== Public functions kept from original ===================== */

bool GSM_SendData(const uint8_t *data, size_t length)
{
   // Todo: implement if needed
}

GSM_ErrorCode GSM_Init(void)
{

    // Board power rails (if applicable)
    nrf_gpio_pin_set(PIN_GSM_PWR);
    nrf_gpio_pin_set(PIN_GSM_EMERG_OFF);

    // Init UART for GSM with our callback
    gsm_uart_init(process_gsm_response);

    // Clean power on and autobaud
    if (GSM_CheckModule())
    {
        device_info.gsm.is_powered_on = true;
        NRF_LOG_INFO("GSM Module is already powered on");
        NRF_LOG_FLUSH();
    }
    else
    {
        device_info.gsm.is_powered_on = false;
        NRF_LOG_INFO("GSM Module is powered off, powering on...");
        NRF_LOG_FLUSH();
        if (!m95_power_on())
        {
            NRF_LOG_INFO("GSM Power On Failed, Retrying...");
            NRF_LOG_FLUSH();
            return GSM_ERROR_PWR_ON_FAILED;
        }
    }

    nrf_gpio_pin_clear(GSM_MCU_DTR);
    nrf_delay_ms(20);
    nrf_gpio_pin_set(GSM_MCU_DTR);
    NRF_LOG_INFO("Device Powered On");
    (void)m95_autobaud_and_lock();

    // Quick sanity probe
    GSM_SendCommand(CMD_ATI);
    GSM_SendCommand(CMD_AT_GSN);

    if (device_info.gsm.imei > 0)
    {
        char imei_str[32];
        snprintf(imei_str, sizeof(imei_str), "%" PRIu64, (uint64_t)device_info.gsm.imei);
        NRF_LOG_INFO("GSM Module IMEI: %s", nrf_log_push(imei_str));
        NRF_LOG_INFO("GSM Module Model: %s", device_info.gsm.model);
        NRF_LOG_INFO("GSM Module Revision: %s", device_info.gsm.Revision);
        NRF_LOG_FLUSH();
        NRF_LOG_INFO("GSM Module Initialization Started...");
        NRF_LOG_FLUSH();
        // Full init sequence

        if (!GSM_SendCommand(CMD_AT_CPIN))
        {
            NRF_LOG_ERROR("SIM not ready, Halting...");
            NRF_LOG_FLUSH();
            return GSM_ERROR_SIM_NOT_READY;
        }
        else
        {
            GSM_SendCommand(CMD_AT_CMEE);
            // GSM_SendCommand(CMD_AT_COPS_SCAN);
            // GSM_SendCommand_uint(CMD_AT_COPS_MANUAL, OPERATOR_JAZZ);
            if (!GSM_SendCommand(CMD_AT_CREG_Q))
            {
                GSM_SendCommand_uint(CMD_AT_CREG_NETWORK_REGISTER, 2); // Enable network registration URC
                GSM_SendCommand_uint(CMD_AT_QBAND_VALUE, 2);           // Set to EGSM 900 / DCS 1800
                nrf_delay_ms(1000);
                GSM_SendCommand(CMD_AT_CFUN_REBOOT); // Reboot to apply band change
                nrf_delay_ms(5000);                  // Wait for reboot
                GSM_SendCommand(CMD_AT_CMEE);
                GSM_SendCommand_uint(CMD_AT_CREG_NETWORK_REGISTER, 2); // Enable network registration URC
                last_cmd_index = GSM_WAIT_CREG;
                while (!device_info.gsm.network_registered)
                {
                    NRF_LOG_INFO("Waiting for Network Registration...");
                    NRF_LOG_FLUSH();
                    nrf_delay_ms(1000);
                }
                NRF_LOG_INFO("Network Registered");
                NRF_LOG_FLUSH();
                nrf_delay_ms(1000);
                if (!GSM_SendCommand(CMD_AT_CSQ))
                {
                    NRF_LOG_ERROR("Poor Signal Quality, Halting...");
                    NRF_LOG_FLUSH();
                    return GSM_ERROR_POOR_SIGNAL;
                }
            }
            else
            {
                NRF_LOG_INFO("Network Registered");
                NRF_LOG_FLUSH();
                GSM_SendCommand(CMD_AT_CSQ);
            }
        }

        GSM_SendCommand(CMD_AT_CGATT_Q);
        if (!device_info.gsm.gprs_attached)
        {
            nrf_delay_ms(3000);
            if (GSM_SendCommand(CMD_AT_CGATT))
            {
                nrf_delay_ms(2000);
                GSM_SendCommand(CMD_AT_CGATT_Q);
                if (!device_info.gsm.gprs_attached)
                {
                    NRF_LOG_ERROR("Failed to attach GPRS, Halting...");
                    NRF_LOG_FLUSH();
                    return GSM_ERROR_GPRS_ATTACH_FAILED;
                }
            }
            else
            {
                NRF_LOG_ERROR("Failed to attach GPRS, Halting...");
                NRF_LOG_FLUSH();
                return GSM_ERROR_GPRS_ATTACH_FAILED;
            }
        }
        NRF_LOG_INFO("GPRS Attached");
        NRF_LOG_FLUSH();
        // PDP context setup
        GSM_SendCommand(CMD_AT_QIDEACT);
        GSM_SendCommand(CMD_AT_QIFGCNT);
        GSM_SendCommand_str(CMD_AT_QICSGP, APN_STR);
        GSM_SendCommand(CMD_AT_QICSGP_Q);
        // GSM_SendCommand(CMD_AT_QIDNSIP);
        GSM_SendCommand(CMD_AT_QIREGAPP);
        if (GSM_SendCommand(CMD_AT_QIACT))
        {
            if (GSM_SendCommand_str(CMD_AT_QPING, "\"8.8.8.8\",4,4"))
            {
                // start pdp context
            }
            GSM_SendCommand(CMD_AT_QILOCIP);
            NRF_LOG_INFO("GSM Module IP Address: %s", device_info.gsm.ip_address);
            NRF_LOG_FLUSH();
            GSM_SendCommand(CMD_AT_QMTOPEN);
               if(!GSM_SendCommand(CMD_AT_QMTOPEN_Q))
               {
                return GSM_ERROR_MQTT_OPEN_FAILED;
            }
            GSM_SendCommand(CMD_AT_QIPROMPT);
            
            GSM_SendCommand(CMD_AT_QMTCONN);
            if(!GSM_SendCommand(CMD_AT_QMTCONN_Q))
            {
                return GSM_ERROR_MQTT_CONNECTION_FAILED;
            }
            if (GSM_SendCommand(CMD_AT_QMTSUB))
            {
                NRF_LOG_INFO("GSM Module Initialization Complete");
                NRF_LOG_FLUSH();
            }
            else
            {
                NRF_LOG_ERROR("Failed to subscribe to MQTT topic, Halting...");
                NRF_LOG_FLUSH();
                return GSM_ERROR_MQTT_SUBSCRIPTION_FAILED;
            }
        }
        else
        {
            GSM_SendCommand(CMD_AT_QICSGP_Q);
            GSM_SendCommand(CMD_AT_QIREGAPP_Q);
            GSM_SendCommand(CMD_AT_QIACT);
            GSM_SendCommand(CMD_AT_CGREG_Q);
            GSM_SendCommand(CMD_AT_CGATT_Q);
            GSM_SendCommand(CMD_AT_QIACT_Q);
            GSM_SendCommand(CMD_AT_CGPADDR);
            return GSM_ERROR_PDP_CONTEXT_ACTIVATION_FAILED;
        }
    }
    else
    {
        NRF_LOG_INFO("Failed to get GSM Module IMEI");
        NRF_LOG_FLUSH();
        return GSM_ERROR_NO_IMEI;
    }
    return GSM_SUCCESS;
}

bool GSM_CheckModule(void)
{
    // Prefer an AT handshake over sampling STATUS/NETLIGHT
    bool ok = m95_check_ready(CMD_AT);
    NRF_LOG_INFO("GSM check (AT): %s", ok ? "OK" : "no response");
    NRF_LOG_FLUSH();
    return ok;
}

bool GSM_Setup(void)
{
    if (!GSM_SendCommand(CMD_AT))
        return false;
    if (!GSM_SendCommand(CMD_ATE0))
        return false;
    if (!GSM_SendCommand(CMD_AT_CMEE))
        return false;
    return true;
}

void GSM_PowerOn(void)
{
    m95_power_on();
    // Wait until the modem answers AT
    for (int i = 0; i < 20; ++i)
    {
        if (m95_check_ready(CMD_AT))
            break;
        nrf_delay_ms(250);
    }
    // Ensure we lock baud/echo settings
    (void)m95_autobaud_and_lock();
}

void GSM_PowerOff(void)
{
    // Try graceful shutdown first
    GSM_SendCommand(CMD_AT_QPOWD);
    // As a fallback (or after QPOWD), assert OFF pulse
    m95_power_off();
}

void GSM_Reset(void)
{
    // Hardware reset if you have the pin; keep as in original
    nrf_gpio_pin_clear(GSM_MCU_RST);
    nrf_delay_ms(100);
    nrf_gpio_pin_set(GSM_MCU_RST);
    nrf_delay_ms(5000);
}

void GSM_EmergencyOff(void)
{
    // Immediate off: assert PWRKEY low long enough; refine as needed for your board
    nrf_gpio_cfg_output(PIN_GSM_PWRKEY);
    nrf_gpio_pin_clear(PIN_GSM_PWRKEY);
}
