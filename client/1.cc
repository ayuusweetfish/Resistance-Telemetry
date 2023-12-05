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

/*
  adapter.set_callback_on_scan_found([] (SimpleBLE::Peripheral p) {
    printf("Found   %s, %d dBm, %s\n", p.address().c_str(), p.rssi(), p.identifier().c_str());
  });
  adapter.set_callback_on_scan_updated([] (SimpleBLE::Peripheral p) {
    printf("Updated %s, %d dBm, %s\n", p.address().c_str(), p.rssi(), p.identifier().c_str());
  });
*/
  auto upd_print = [] (SimpleBLE::Peripheral p) {
    if (p.identifier() == "RC") {
      auto mfr_data = p.manufacturer_data();
      for (auto &[x, y] : mfr_data) {
        printf("%04x", x);
        for (int i = 0; i < y.length(); i++) printf(" %02x", (uint8_t)y[i]);
        putchar('\n');
      }
    }
  };
  adapter.set_callback_on_scan_found(upd_print);
  adapter.set_callback_on_scan_updated(upd_print);

  adapter.scan_for(10000);

  return 0;
}
