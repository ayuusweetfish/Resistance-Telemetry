// c++ % -ISimpleBLE/simpleble/include -Ibuild_simpleble/export build_simpleble/lib/libsimpleble.a -std=c++17 -framework CoreBluetooth -framework Foundation

#include "simpleble/SimpleBLE.h"

#include <cassert>
#include <cstdio>

int main() {
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
        // printf("%02x %02x", x & 0xff, (x >> 8) & 0xff);
        // for (int i = 0; i < y.length(); i++) printf(" %02x", (uint8_t)y[i]);
        // putchar('\n');
        uint8_t timestamp = x & 0xff;
        uint32_t value0 = 
          ((uint32_t)((x >> 8) & 0xff) << 16) |
          ((uint32_t)(y[0] & 0xff) << 8) |
          (y[1] & 0xff);
        // printf("t = %3d: %9.6lf\n", timestamp, (double)((int32_t)(value0 << 8) >> 8) / 0x7fffff);
        printf("t = %3d: %06x\n", timestamp, value0);
      }
    }
  };
  adapter.set_callback_on_scan_found(upd_print);
  adapter.set_callback_on_scan_updated(upd_print);

  printf("Starting scan\n");
  adapter.scan_for(100000);

  return 0;
}
