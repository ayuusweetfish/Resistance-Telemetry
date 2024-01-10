// c++ % -ISimpleBLE/simpleble/include -Ibuild_simpleble/export build_simpleble/lib/libsimpleble.a -Ilibmicrohttpd-0.9.77/src/include libmicrohttpd-0.9.77/src/microhttpd/.libs/libmicrohttpd.a -std=c++17 -framework CoreBluetooth -framework Foundation

#include "simpleble/SimpleBLE.h"
#include "microhttpd.h"

#include <cassert>
#include <cerrno>
#include <climits>    // INT_MAX
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>

#include <algorithm>  // std::upper_bound
#include <deque>
#include <mutex>
#include <sstream>    // std::ostringstream
#include <utility>    // std::pair
#include <vector>

#define USE_BUILTIN_INDEX_HTML 0
#if USE_BUILTIN_INDEX_HTML
// xxd -i index.html > index.html.h
#include "index.html.h"
// unsigned char index_html[]
// unsigned int index_html_len
#else
unsigned char index_html[4000000];
unsigned int index_html_len;
#endif

std::deque<std::pair<int, int>> readings;
std::mutex readings_mutex;

int main() {
#if !USE_BUILTIN_INDEX_HTML
  {
    FILE *f = fopen("index.html", "rb");
    fseek(f, 0, SEEK_END);
    index_html_len = ftell(f);
    fseek(f, 0, SEEK_SET);
    fread(index_html, index_html_len, 1, f);
    fclose(f);
  }
#endif

if (1) {
  if (!SimpleBLE::Adapter::bluetooth_enabled()) {
    printf("Bluetooth not enabled\n");
    return 1;
  }

  auto adapters = SimpleBLE::Adapter::get_adapters();
  if (adapters.empty()) {
    printf("No adapter found\n");
    return 1;
  }
  auto adapter = adapters[0];

  auto upd_print = [] (SimpleBLE::Peripheral p) {
    if (p.identifier() == "RC") {
      auto mfr_data = p.manufacturer_data();
      for (auto &[x, y] : mfr_data) {
        uint8_t payload[16];
        payload[0] = x & 0xff;
        payload[1] = (x >> 8) & 0xff;
        for (int i = 0; i < y.length(); i++) payload[i + 2] = (uint8_t)y[i];
        // for (int i = 0; i < y.length() + 2; i++) printf(" %02x", payload[i]); putchar('\n');
        auto read_bits = [] (const uint8_t *buffer, size_t start, size_t length) -> uint32_t {
          uint32_t result = 0;
          for (int i = length - 1; i >= 0; i--) {
            int index = (start + i) / 8;
            int bitpos = (start + i) % 8;
            result = (result << 1) | ((buffer[index] >> bitpos) & 1);
          }
          return result;
        };
        uint32_t start_timestamp = read_bits(payload, 0, 10);
        for (int i = 0; i < 5; i++) {
          uint32_t timestamp = (start_timestamp - i) & ((1 << 10) - 1);
          uint32_t value0 = read_bits(payload, 10 + i * 22, 22);
          double normalized = (double)((int32_t)(value0 << 10) >> 10) / (1 << 22);
          double resistance = 4990 * (1.0 / (0.5 + normalized) - 1);
          fprintf(stderr, "t = %4u: ADC = %06x  R = %12.4lf\n", timestamp, value0, resistance);
        }
      }
    }
  };
  adapter.set_callback_on_scan_found(upd_print);
  adapter.set_callback_on_scan_updated(upd_print);

  printf("Starting scan\n");
  adapter.scan_start();
  while (1) sleep(1);
}

  auto request_handler = [] (
    void *cls,
    struct MHD_Connection *conn,
    const char *url,
    const char *method,
    const char *version,
    const char *upload_data,
    size_t *upload_data_size,
    void **ptr)
  -> enum MHD_Result {
    enum MHD_Result result;

    if (strcmp(method, "GET") != 0) {
      struct MHD_Response *resp = MHD_create_response_from_buffer(0, NULL, MHD_RESPMEM_PERSISTENT);
      result = MHD_queue_response(conn, MHD_HTTP_METHOD_NOT_ALLOWED /* 405 */, resp);
      MHD_destroy_response(resp);
      return result;
    }

    if (strcmp(url, "/") == 0) {
      struct MHD_Response *resp = MHD_create_response_from_buffer(
        index_html_len, (void *)index_html,
        MHD_RESPMEM_PERSISTENT);
      MHD_add_response_header(resp, "Content-Type", "text/html; charset=UTF-8");
      result = MHD_queue_response(conn, MHD_HTTP_OK /* 200 */, resp);
      MHD_destroy_response(resp);
      return result;
    }

    if (strcmp(url, "/data") == 0) {
      const char *since_str = MHD_lookup_connection_value(conn, MHD_GET_ARGUMENT_KIND, "since");
      int since;
      errno = 0;
      if (since_str == NULL || (since = (int)strtol(since_str, NULL, 0), errno) != 0) {
        struct MHD_Response *resp = MHD_create_response_from_buffer(0, NULL, MHD_RESPMEM_PERSISTENT);
        result = MHD_queue_response(conn, MHD_HTTP_BAD_REQUEST /* 400 */, resp);
        MHD_destroy_response(resp);
        return result;
      }
      // Read the values
      readings_mutex.lock();
      auto itr = std::upper_bound(readings.begin(), readings.end(),
        std::make_pair(since, INT_MAX));
      std::vector<std::pair<int, int>> returned_values(itr, readings.end());
      readings_mutex.unlock();
      // Convert to space-separated string
      std::ostringstream ss;
      for (auto entry : returned_values) {
        if (ss.tellp() != 0) ss << ' ';
        ss << entry.first << ' ' << entry.second;
      }
      struct MHD_Response *resp = MHD_create_response_from_buffer(
        ss.tellp(), (void *)ss.str().c_str(),
        MHD_RESPMEM_MUST_COPY);
      MHD_add_response_header(resp, "Content-Type", "text/plain");
      result = MHD_queue_response(conn, MHD_HTTP_OK /* 200 */, resp);
      MHD_destroy_response(resp);
      return result;
    }

    struct MHD_Response *resp = MHD_create_response_from_buffer(0, NULL, MHD_RESPMEM_PERSISTENT);
    result = MHD_queue_response(conn, MHD_HTTP_NOT_FOUND /* 404 */, resp);
    MHD_destroy_response(resp);
    return result;
  };

  struct MHD_Daemon *daemon = MHD_start_daemon(
    MHD_USE_SELECT_INTERNALLY,
    24017,
    (MHD_AcceptPolicyCallback)NULL, (void *)NULL,
    (MHD_AccessHandlerCallback)+request_handler, (void *)NULL,
    MHD_OPTION_END
  );
  printf("http://localhost:24017/\n");

  int x = 0;
  while (1) {
    x++;
    readings_mutex.lock();
    readings.push_back({x, 1000 + x * 20 + (99999999 / x) * 477 % 997});
    readings_mutex.unlock();
    usleep(200000);
  }
  MHD_stop_daemon(daemon);

  return 0;
}
