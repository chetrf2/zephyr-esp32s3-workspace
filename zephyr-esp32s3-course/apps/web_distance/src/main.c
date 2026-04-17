#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/dhcpv4.h>
#include <zephyr/net/http/server.h>
#include <zephyr/net/http/service.h>
#include <zephyr/net/http/status.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/data/json.h>
#include <string.h>

LOG_MODULE_REGISTER(web_distance, LOG_LEVEL_INF);

#define HTTP_PORT 80
#define ADC_CHANNELS 8
#define WIFI_CONNECT_TIMEOUT K_SECONDS(15)
#define IPV4_READY_TIMEOUT K_SECONDS(20)
#define WIFI_CONNECT_RETRIES 3

#define US100_TRIGGER       0x55
#define US100_REPLY_BYTES   2
#define US100_RX_TIMEOUT_MS 100

/* Shared state between ISR callback and thread */
static K_SEM_DEFINE(us100_rx_sem, 0, 1);
static uint8_t  us100_rx_buf[US100_REPLY_BYTES];
static uint8_t  us100_rx_count;

static void us100_uart_isr(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	if (!uart_irq_update(dev)) {
		return;
	}

	while (uart_irq_rx_ready(dev)) {
		uint8_t c;

		if (uart_fifo_read(dev, &c, 1) != 1) {
			break;
		}

		if (us100_rx_count < US100_REPLY_BYTES) {
			us100_rx_buf[us100_rx_count++] = c;
		}

		if (us100_rx_count == US100_REPLY_BYTES) {
			k_sem_give(&us100_rx_sem);
		}
	}
}

static struct net_mgmt_event_callback wifi_cb;
static struct net_mgmt_event_callback ipv4_cb;

static K_SEM_DEFINE(wifi_connected, 0, 1);
static K_SEM_DEFINE(ipv4_ready, 0, 1);
static int wifi_connect_result;

#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);

static const struct device *const sht40 = DEVICE_DT_GET(DT_NODELABEL(sht4x_0));
static const struct spi_dt_spec mcp3008 = SPI_DT_SPEC_GET(
	DT_NODELABEL(mcp3008),
	SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
	0);

static const uint8_t index_html[] =
"<!doctype html>\n"
"<html lang=\"en\">\n"
"<head>\n"
"  <meta charset=\"utf-8\" />\n"
"  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\" />\n"
"  <title>Zephyr Web Controller</title>\n"
"  <style>\n"
"    :root {\n"
"      --bg: #0b1020;\n"
"      --card: rgba(15,23,42,0.8);\n"
"      --accent: #38bdf8;\n"
"      --accent2: #22c55e;\n"
"      --text: #e2e8f0;\n"
"      --muted: #94a3b8;\n"
"    }\n"
"    * { box-sizing: border-box; }\n"
"    body {\n"
"      margin: 0;\n"
"      min-height: 100vh;\n"
"      font-family: \"Space Grotesk\", \"Segoe UI\", system-ui, sans-serif;\n"
"      color: var(--text);\n"
"      background:\n"
"        radial-gradient(900px 500px at 10% -10%, rgba(56,189,248,0.22), transparent 60%),\n"
"        radial-gradient(800px 400px at 110% 10%, rgba(34,197,94,0.16), transparent 55%),\n"
"        linear-gradient(135deg, #0b1020, #0f172a);\n"
"      display: grid;\n"
"      place-items: center;\n"
"      padding: 24px;\n"
"    }\n"
"    .grid {\n"
"      width: 100%;\n"
"      max-width: 900px;\n"
"      display: grid;\n"
"      grid-template-columns: repeat(auto-fit, minmax(240px, 1fr));\n"
"      gap: 16px;\n"
"    }\n"
"    .card {\n"
"      background: var(--card);\n"
"      border: 1px solid rgba(148,163,184,0.2);\n"
"      border-radius: 16px;\n"
"      padding: 18px;\n"
"      box-shadow: 0 16px 40px rgba(0,0,0,0.35);\n"
"      backdrop-filter: blur(6px);\n"
"    }\n"
"    h1 { margin: 0 0 8px; font-size: 28px; }\n"
"    h2 { margin: 0 0 12px; font-size: 16px; color: var(--muted); }\n"
"    .value { font-size: 26px; font-weight: 600; }\n"
"    .adc {\n"
"      display: grid;\n"
"      grid-template-columns: repeat(2, 1fr);\n"
"      gap: 8px;\n"
"      font-family: ui-monospace, SFMono-Regular, Menlo, monospace;\n"
"      font-size: 13px;\n"
"    }\n"
"    .toggle {\n"
"      display: flex;\n"
"      align-items: center;\n"
"      justify-content: space-between;\n"
"      gap: 10px;\n"
"    }\n"
"    button {\n"
"      background: linear-gradient(90deg, var(--accent), var(--accent2));\n"
"      color: #0b1020;\n"
"      border: 0;\n"
"      border-radius: 999px;\n"
"      padding: 8px 14px;\n"
"      font-weight: 700;\n"
"      cursor: pointer;\n"
"    }\n"
"    .pill { color: var(--muted); font-size: 12px; }\n"
"  </style>\n"
"</head>\n"
"<body>\n"
"  <div class=\"grid\">\n"
"    <div class=\"card\" style=\"grid-column: 1 / -1;\">\n"
"      <h1>Zephyr Web Controller</h1>\n"
"      <div class=\"pill\">Live sensor + ADC values and LED control</div>\n"
"    </div>\n"
"    <div class=\"card\">\n"
"      <h2>SHT40 Temperature</h2>\n"
"      <div id=\"temp\" class=\"value\">--.- C</div>\n"
"    </div>\n"
"    <div class=\"card\">\n"
"      <h2>SHT40 Humidity</h2>\n"
"      <div id=\"rh\" class=\"value\">--.- %</div>\n"
"    </div>\n"
"    <div class=\"card\">\n"
"      <h2>US-100 Distance</h2>\n"
"      <div id=\"distance\" class=\"value\">---- mm</div>\n"
"    </div>\n"
"    <div class=\"card\">\n"
"      <h2>ADC Channels</h2>\n"
"      <div id=\"adc\" class=\"adc\"></div>\n"
"    </div>\n"
"    <div class=\"card\">\n"
"      <h2>LED Control</h2>\n"
"      <div class=\"toggle\">\n"
"        <div>LED0</div>\n"
"        <button onclick=\"setLed(0)\">Toggle</button>\n"
"      </div>\n"
"      <div class=\"toggle\" style=\"margin-top: 10px;\">\n"
"        <div>LED1</div>\n"
"        <button onclick=\"setLed(1)\">Toggle</button>\n"
"      </div>\n"
"    </div>\n"
"  </div>\n"
"  <script>\n"
"    const state = { led0: false, led1: false };\n"
"    async function fetchTelemetry() {\n"
"      const res = await fetch('/api/telemetry');\n"
"      if (!res.ok) return;\n"
"      const data = await res.json();\n"
"      document.getElementById('temp').textContent = data.temp_c.toFixed(1) + ' C';\n"
"      document.getElementById('rh').textContent = data.rh.toFixed(1) + ' %';\n"
"      document.getElementById('distance').textContent = (data.distance_mm ?? 0) + ' mm';\n"
"      const adc = data.adc || [];\n"
"      document.getElementById('adc').innerHTML = adc.map((v,i)=>`CH${i}: ${v}`).join('<br/>');\n"
"    }\n"
"    async function setLed(idx) {\n"
"      const key = idx === 0 ? 'led0' : 'led1';\n"
"      state[key] = !state[key];\n"
"      await fetch('/api/led', {\n"
"        method: 'POST',\n"
"        headers: { 'Content-Type': 'application/json' },\n"
"        body: JSON.stringify({ led: idx, state: state[key] ? 1 : 0 })\n"
"      });\n"
"    }\n"
"    setInterval(fetchTelemetry, 250);\n"
"    fetchTelemetry();\n"
"  </script>\n"
"</body>\n"
"</html>\n";

static struct http_resource_detail_static index_html_resource_detail = {
	.common = {
			.type = HTTP_RESOURCE_TYPE_STATIC,
			.bitmask_of_supported_http_methods = BIT(HTTP_GET),
			.content_type = "text/html; charset=utf-8",
		},
	.static_data = index_html,
	.static_data_len = sizeof(index_html) - 1,
};

static uint16_t web_port = HTTP_PORT;
HTTP_SERVICE_DEFINE(web_distance_service, NULL, &web_port, 1, 10, NULL);

HTTP_RESOURCE_DEFINE(index_html_resource, web_distance_service, "/",
		     &index_html_resource_detail);
HTTP_RESOURCE_DEFINE(index_html_resource_alias, web_distance_service, "/index.html",
		     &index_html_resource_detail);

struct led_command {
	int led;
	int state;
};

static const struct json_obj_descr led_command_descr[] = {
	JSON_OBJ_DESCR_PRIM(struct led_command, led, JSON_TOK_NUMBER),
	JSON_OBJ_DESCR_PRIM(struct led_command, state, JSON_TOK_NUMBER),
};

static const struct http_header json_headers[] = {
	{ .name = "Content-Type", .value = "application/json" },
};

static int read_sht40(float *temp_c, float *rh)
{
	struct sensor_value val;

	if (!device_is_ready(sht40)) {
		return -ENODEV;
	}
	if (sensor_sample_fetch(sht40) < 0) {
		return -EIO;
	}
	if (sensor_channel_get(sht40, SENSOR_CHAN_AMBIENT_TEMP, &val) < 0) {
		return -EIO;
	}
	*temp_c = sensor_value_to_double(&val);
	if (sensor_channel_get(sht40, SENSOR_CHAN_HUMIDITY, &val) < 0) {
		return -EIO;
	}
	*rh = sensor_value_to_double(&val);
	return 0;
}

static int read_us100(uint16_t *distance_mm)
{
	const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart1));

	if (!device_is_ready(uart)) {
		return -ENODEV;
	}

	/* Reset RX state then fire trigger; ISR will collect the reply */
	us100_rx_count = 0;
	k_sem_reset(&us100_rx_sem);
	uart_poll_out(uart, US100_TRIGGER);

	/* Block (not spin) until ISR signals 2 bytes received, or timeout */
	if (k_sem_take(&us100_rx_sem, K_MSEC(US100_RX_TIMEOUT_MS)) != 0) {
		return -ETIMEDOUT;
	}

	*distance_mm = ((uint16_t)us100_rx_buf[0] << 8) | us100_rx_buf[1];
	return 0;
}

static int read_mcp3008(uint8_t channel, uint16_t *value)
{
	uint8_t tx[3] = { 0x01, (uint8_t)(0x80 | (channel << 4)), 0x00 };
	uint8_t rx[3] = { 0 };
	const struct spi_buf tx_buf = { .buf = tx, .len = sizeof(tx) };
	const struct spi_buf rx_buf = { .buf = rx, .len = sizeof(rx) };
	const struct spi_buf_set tx_set = { .buffers = &tx_buf, .count = 1 };
	const struct spi_buf_set rx_set = { .buffers = &rx_buf, .count = 1 };

	if (channel > 7) {
		return -EINVAL;
	}
	if (!spi_is_ready_dt(&mcp3008)) {
		return -ENODEV;
	}

	int ret = spi_transceive_dt(&mcp3008, &tx_set, &rx_set);
	if (ret < 0) {
		return ret;
	}

	*value = (uint16_t)(((rx[1] & 0x03) << 8) | rx[2]);
	return 0;
}

static int read_adc_all(uint16_t *values, size_t count)
{
	for (size_t i = 0; i < count; i++) {
		if (read_mcp3008((uint8_t)i, &values[i]) < 0) {
			return -EIO;
		}
	}
	return 0;
}

static int telemetry_handler(struct http_client_ctx *client, enum http_data_status status,
			     uint8_t *buffer, size_t len, struct http_response_ctx *response_ctx,
			     void *user_data)
{
	ARG_UNUSED(client);
	ARG_UNUSED(buffer);
	ARG_UNUSED(len);
	ARG_UNUSED(user_data);

	static char body[256];
	float temp_c = 0.0f;
	float rh = 0.0f;
	uint16_t adc[ADC_CHANNELS] = { 0 };
	uint16_t distance_mm = 0;
	int ret;

	if (status != HTTP_SERVER_DATA_FINAL) {
		return 0;
	}

	ret = read_sht40(&temp_c, &rh);
	if (ret < 0) {
		LOG_WRN("SHT40 read failed (%d)", ret);
		temp_c = -1.0f;
		rh = -1.0f;
	}
	if (read_adc_all(adc, ADC_CHANNELS) < 0) {
		LOG_WRN("MCP3008 read failed");
		memset(adc, 0, sizeof(adc));
	}
	if (read_us100(&distance_mm) < 0) {
		LOG_WRN("US-100 read failed");
		distance_mm = 0;
	}

	ret = snprintk(
		body, sizeof(body),
		"{\"temp_c\":%.1f,\"rh\":%.1f,\"distance_mm\":%u,\"adc\":[%u,%u,%u,%u,%u,%u,%u,%u]}",
		(double)temp_c, (double)rh,
		distance_mm,
		adc[0], adc[1], adc[2], adc[3],
		adc[4], adc[5], adc[6], adc[7]);

	if (ret < 0) {
		return ret;
	}

	LOG_INF("Telemetry: %.1f C, %.1f %%, %u mm",
		(double)temp_c, (double)rh, distance_mm);

	response_ctx->status = HTTP_200_OK;
	response_ctx->headers = json_headers;
	response_ctx->header_count = ARRAY_SIZE(json_headers);
	response_ctx->body = (uint8_t *)body;
	response_ctx->body_len = (size_t)ret;
	response_ctx->final_chunk = true;

	return 0;
}

static struct http_resource_detail_dynamic telemetry_resource_detail = {
	.common = {
			.type = HTTP_RESOURCE_TYPE_DYNAMIC,
			.bitmask_of_supported_http_methods = BIT(HTTP_GET),
		},
	.cb = telemetry_handler,
	.user_data = NULL,
};

static int led_handler(struct http_client_ctx *client, enum http_data_status status,
		       uint8_t *buffer, size_t len, struct http_response_ctx *response_ctx,
		       void *user_data)
{
	ARG_UNUSED(client);
	ARG_UNUSED(user_data);

	static uint8_t post_buf[64];
	static size_t cursor;
	struct led_command cmd;
	int ret;

	if (status == HTTP_SERVER_DATA_ABORTED) {
		cursor = 0;
		return 0;
	}

	if (len + cursor > sizeof(post_buf)) {
		cursor = 0;
		return -ENOMEM;
	}

	memcpy(post_buf + cursor, buffer, len);
	cursor += len;

	if (status == HTTP_SERVER_DATA_FINAL) {
		ret = json_obj_parse(post_buf, cursor, led_command_descr,
				     ARRAY_SIZE(led_command_descr), &cmd);
		cursor = 0;

		if (ret < 0) {
			return -EINVAL;
		}

		if (cmd.led == 0 && gpio_is_ready_dt(&led0)) {
			(void)gpio_pin_set_dt(&led0, cmd.state ? 1 : 0);
		} else if (cmd.led == 1 && gpio_is_ready_dt(&led1)) {
			(void)gpio_pin_set_dt(&led1, cmd.state ? 1 : 0);
		}

		response_ctx->status = HTTP_200_OK;
		response_ctx->headers = json_headers;
		response_ctx->header_count = ARRAY_SIZE(json_headers);
		response_ctx->body = (uint8_t *)"{\"ok\":true}";
		response_ctx->body_len = strlen((char *)response_ctx->body);
		response_ctx->final_chunk = true;
	}

	return 0;
}

static struct http_resource_detail_dynamic led_resource_detail = {
	.common = {
			.type = HTTP_RESOURCE_TYPE_DYNAMIC,
			.bitmask_of_supported_http_methods = BIT(HTTP_POST),
		},
	.cb = led_handler,
	.user_data = NULL,
};

HTTP_RESOURCE_DEFINE(telemetry_resource, web_distance_service, "/api/telemetry",
		     &telemetry_resource_detail);
HTTP_RESOURCE_DEFINE(led_resource, web_distance_service, "/api/led",
		     &led_resource_detail);

static void wifi_event_handler(struct net_mgmt_event_callback *cb,
			       uint32_t mgmt_event, struct net_if *iface)
{
	const struct wifi_status *status = cb->info;

	ARG_UNUSED(iface);

	if (mgmt_event != NET_EVENT_WIFI_CONNECT_RESULT) {
		return;
	}

	if (status == NULL) {
		wifi_connect_result = -EIO;
		LOG_ERR("Wi-Fi connect result missing status");
		k_sem_give(&wifi_connected);
		return;
	}

	wifi_connect_result = status->status;

	if (status->status) {
		LOG_ERR("Wi-Fi connect failed (%d)", status->status);
		k_sem_give(&wifi_connected);
		return;
	}

	LOG_INF("Wi-Fi connected");
	k_sem_give(&wifi_connected);
}

static void ipv4_event_handler(struct net_mgmt_event_callback *cb,
			       uint32_t mgmt_event, struct net_if *iface)
{
	char addr_buf[NET_IPV4_ADDR_LEN];

	ARG_UNUSED(cb);

	if (mgmt_event != NET_EVENT_IPV4_ADDR_ADD) {
		return;
	}

	for (int i = 0; i < NET_IF_MAX_IPV4_ADDR; i++) {
		if (iface->config.ip.ipv4->unicast[i].ipv4.addr_type != NET_ADDR_DHCP) {
			continue;
		}

		LOG_INF("IPv4 address: %s",
			net_addr_ntop(AF_INET,
				      &iface->config.ip.ipv4->unicast[i].ipv4.address.in_addr,
				      addr_buf, sizeof(addr_buf)));
		LOG_INF("Browser: http://%s/", addr_buf);
		k_sem_give(&ipv4_ready);
		break;
	}
}

static int connect_wifi_and_wait_for_ip(void)
{
	struct net_if *iface = net_if_get_default();
	struct wifi_connect_req_params params = { 0 };
	int ret;
	bool callbacks_registered;

	if (iface == NULL) {
		LOG_ERR("No default network interface");
		return -ENODEV;
	}

	callbacks_registered = wifi_cb.handler != NULL && ipv4_cb.handler != NULL;
	if (!callbacks_registered) {
		net_mgmt_init_event_callback(&wifi_cb, wifi_event_handler,
					     NET_EVENT_WIFI_CONNECT_RESULT);
		net_mgmt_add_event_callback(&wifi_cb);

		net_mgmt_init_event_callback(&ipv4_cb, ipv4_event_handler,
					     NET_EVENT_IPV4_ADDR_ADD);
		net_mgmt_add_event_callback(&ipv4_cb);
	}

	params.ssid = CONFIG_WIFI_CREDENTIALS_STATIC_SSID;
	params.ssid_length = strlen(CONFIG_WIFI_CREDENTIALS_STATIC_SSID);
	params.psk = CONFIG_WIFI_CREDENTIALS_STATIC_PASSWORD;
	params.psk_length = strlen(CONFIG_WIFI_CREDENTIALS_STATIC_PASSWORD);
	params.security = WIFI_SECURITY_TYPE_PSK;

	for (int attempt = 1; attempt <= WIFI_CONNECT_RETRIES; attempt++) {
		k_sem_reset(&wifi_connected);
		k_sem_reset(&ipv4_ready);
		wifi_connect_result = -ETIMEDOUT;

		LOG_INF("Connecting to Wi-Fi SSID \"%s\" (attempt %d/%d)...",
			CONFIG_WIFI_CREDENTIALS_STATIC_SSID, attempt, WIFI_CONNECT_RETRIES);

		net_if_up(iface);
		ret = net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &params, sizeof(params));
		if (ret < 0) {
			LOG_ERR("Wi-Fi connect request failed (%d)", ret);
			k_sleep(K_SECONDS(1));
			continue;
		}

		ret = k_sem_take(&wifi_connected, WIFI_CONNECT_TIMEOUT);
		if (ret != 0) {
			LOG_ERR("Wi-Fi connect timed out after %d seconds",
				(int)(k_ticks_to_ms_floor32(WIFI_CONNECT_TIMEOUT.ticks) / 1000));
			(void)net_mgmt(NET_REQUEST_WIFI_DISCONNECT, iface, NULL, 0);
			k_sleep(K_SECONDS(1));
			continue;
		}

		if (wifi_connect_result != 0) {
			LOG_ERR("Wi-Fi connect failed (%d), retrying", wifi_connect_result);
			(void)net_mgmt(NET_REQUEST_WIFI_DISCONNECT, iface, NULL, 0);
			k_sleep(K_SECONDS(1));
			continue;
		}

		LOG_INF("Starting DHCPv4...");
		net_dhcpv4_start(iface);
		ret = k_sem_take(&ipv4_ready, IPV4_READY_TIMEOUT);
		if (ret != 0) {
			LOG_ERR("DHCP timed out after %d seconds",
				(int)(k_ticks_to_ms_floor32(IPV4_READY_TIMEOUT.ticks) / 1000));
			net_dhcpv4_stop(iface);
			(void)net_mgmt(NET_REQUEST_WIFI_DISCONNECT, iface, NULL, 0);
			k_sleep(K_SECONDS(1));
			continue;
		}

		return 0;
	}

	return -ETIMEDOUT;
}

static void init_us100(void)
{
	const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart1));

	if (!device_is_ready(uart)) {
		LOG_ERR("US-100: uart1 not ready");
		return;
	}

	uart_irq_callback_user_data_set(uart, us100_uart_isr, NULL);
	uart_irq_rx_enable(uart);
	LOG_INF("US-100: ready on uart1 (GPIO4=TX, GPIO5=RX)");
}

static void init_leds(void)
{
	if (gpio_is_ready_dt(&led0)) {
		(void)gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
	}
	if (gpio_is_ready_dt(&led1)) {
		(void)gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);
	}
}

int main(void)
{
	init_leds();
	init_us100();

	if (!device_is_ready(sht40)) {
		LOG_WRN("SHT40 not ready");
	}
	if (!spi_is_ready_dt(&mcp3008)) {
		LOG_WRN("MCP3008 not ready");
	}

	if (connect_wifi_and_wait_for_ip() == 0) {
		LOG_INF("Starting HTTP server on port %u", web_port);
		http_server_start();
	}

	return 0;
}
