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
#include <iomanip>    // std::setprecision
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

std::deque<std::pair<int, double>> readings;
std::mutex readings_mutex;
int timestamp_base = 0;
bool is_paused = false;

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

  if (!SimpleBLE::Adapter::bluetooth_enabled()) {
    printf("Bluetooth not enabled\n");
    puts("Press Enter to exit"); getchar();
    return 1;
  }

  auto adapters = SimpleBLE::Adapter::get_adapters();
  if (adapters.empty()) {
    printf("No Bluetooth adapter found\n");
    puts("Press Enter to exit"); getchar();
    return 1;
  }
  auto adapter = adapters[0];

  // Add a value to the data. This requires `readings_mutex` to be held.
  auto add_value = [] (uint32_t timestamp, uint32_t raw_adc) -> void {
    if (is_paused) return;

    // Calculate the real timestamp
    int real_timestamp = (int)timestamp + timestamp_base;
    int max_timestamp = (readings.empty() ? 0 : readings.back().first);
    if (abs(real_timestamp - max_timestamp) > 50) {
      // Out of range. Timestamp base needs to be adjusted.
      // Check whether wrapping around (both directions, as data can be out-of-order).
      if (abs((real_timestamp - 1024) - max_timestamp) <= 50) {
        timestamp_base -= 1024;
      } else if (abs((real_timestamp + 1024) - max_timestamp) <= 50) {
        timestamp_base += 1024;
      } else {  // Otherwise, simply treat the timestamp as the start of the new base.
        timestamp_base = (max_timestamp + 1) - (int)timestamp;
      }
      real_timestamp = (int)timestamp + timestamp_base;
    }

    // Calculate resistance value
    int32_t value_signed = (int32_t)(raw_adc << 10) >> 10;
    double normalized = (double)value_signed / (1 << 22);
  /*
    int32_t range_min = -0x1fd200;  // 0x202e00
    int32_t range_max =  0x1fd000;  // 0x1fd000
    double normalized =
      (double)(value_signed - range_min) / (range_max - range_min) - 0.5;
    if (normalized < -0.5) normalized = -0.5;
    if (normalized >  0.5) normalized =  0.5;
  */
    double resistance = 4990 * (1.0 / (0.5 + normalized) - 1);
    fprintf(stderr, "t = %4u (%6d): ADC =%8d  R = %12.4lf\n",
      timestamp, real_timestamp, value_signed, resistance);
    auto insert_pos = std::lower_bound(
      readings.begin(), readings.end(),
      std::make_pair(real_timestamp, -1.0));
    if (insert_pos == readings.end() || insert_pos->first != real_timestamp)
      readings.insert(insert_pos, {real_timestamp, resistance});
  };

  auto upd_print = [add_value] (SimpleBLE::Peripheral p) {
    if (p.identifier() == "RT") {
      auto mfr_data = p.manufacturer_data();
      readings_mutex.lock();
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
        for (int i = 5 - 1; i >= 0; i--) {
          uint32_t timestamp = (start_timestamp - i) & ((1 << 10) - 1);
          if (timestamp >= 1000) continue;
          add_value(timestamp, read_bits(payload, 10 + i * 22, 22));
        }
      }
      readings_mutex.unlock();
    }
  };
  adapter.set_callback_on_scan_found(upd_print);
  adapter.set_callback_on_scan_updated(upd_print);

  printf("Starting scan\n");
  adapter.scan_start();

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

    if (strcmp(method, "GET") == 0 && strcmp(url, "/") == 0) {
      struct MHD_Response *resp = MHD_create_response_from_buffer(
        index_html_len, (void *)index_html,
        MHD_RESPMEM_PERSISTENT);
      MHD_add_response_header(resp, "Content-Type", "text/html; charset=UTF-8");
      result = MHD_queue_response(conn, MHD_HTTP_OK /* 200 */, resp);
      MHD_destroy_response(resp);
      return result;
    }

    if (strcmp(method, "GET") == 0 && strcmp(url, "/data") == 0) {
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
      auto itr = std::lower_bound(readings.begin(), readings.end(),
        std::make_pair(since, std::numeric_limits<double>::min()));
      std::vector<std::pair<int, double>> returned_values(itr, readings.end());
      readings_mutex.unlock();
      // Convert to space-separated string
      std::ostringstream ss;
      for (auto entry : returned_values) {
        if (ss.tellp() != 0) ss << ' ';
        ss << entry.first << ' '
           << std::fixed << std::setprecision(12) << entry.second;
      }
      struct MHD_Response *resp = MHD_create_response_from_buffer(
        ss.tellp(), (void *)ss.str().c_str(),
        MHD_RESPMEM_MUST_COPY);
      MHD_add_response_header(resp, "Content-Type", "text/plain");
      result = MHD_queue_response(conn, MHD_HTTP_OK /* 200 */, resp);
      MHD_destroy_response(resp);
      return result;
    }

    if (strcmp(method, "POST") == 0 && strcmp(url, "/clear") == 0) {
      readings_mutex.lock();
      readings.clear();
      timestamp_base = 0;
      timestamp_base = -5000; // Force a reset after clear
      fputs("Cleared\n", stderr);
      readings_mutex.unlock();
      struct MHD_Response *resp = MHD_create_response_from_buffer(0, NULL, MHD_RESPMEM_PERSISTENT);
      result = MHD_queue_response(conn, MHD_HTTP_OK /* 200 */, resp);
      MHD_destroy_response(resp);
      return result;
    }

    if (strcmp(method, "POST") == 0 &&
        (strcmp(url, "/pause") == 0 || strcmp(url, "/resume") == 0)) {
      readings_mutex.lock();
      is_paused = (strcmp(url, "/pause") == 0);
      if (is_paused) timestamp_base = -5000;  // Force a reset at resume
      fputs(is_paused ? "Paused\n" : "Resumed\n", stderr);
      readings_mutex.unlock();
      struct MHD_Response *resp = MHD_create_response_from_buffer(0, NULL, MHD_RESPMEM_PERSISTENT);
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
  if (daemon == NULL) {
    adapter.scan_stop();
    printf("Cannot start HTTP server\n");
    puts("Press Enter to exit"); getchar();
    return 1;
  }
  printf("http://localhost:24017/\n");

#if defined(WIN32) || defined(_WIN32)
  system("start http://localhost:24017/");
#elif __linux__
  system("xdg-open http://localhost:24017/");
#elif __APPLE__
  system("open http://localhost:24017/");
#endif

  auto rand = [] () -> uint32_t {
    static uint32_t seed = 240111;
    return (seed = (seed * 1103515245 + 12345) & 0x7fffffff);
  };
  int x = 0;
  while (1) {
    x++;
    if ((rand() >> 4) % 2 != 0) {
      if (rand() % 41 == 0) x = (rand() >> 7) & 1023;
      if ((rand() >> 4) % 2 == 0) {
        add_value((x - 4) & 1023, 0x400000 - 2000 + rand() % 997);
        add_value((x - 3) & 1023, 0x400000 - 2000 + rand() % 997);
      }
      add_value(x & 1023, 2000 + rand() % 997);
    }
    usleep(200000);
  }
  while (1) sleep(1);
  MHD_stop_daemon(daemon);

  return 0;
}
