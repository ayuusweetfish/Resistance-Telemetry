import asyncio
import bleak
from bleak import BleakScanner, BleakClient
import datetime

async def main():
  while True:
    devices = await BleakScanner.discover(timeout=0.2, return_adv=True)
    found = False
    for k, (d, adv) in devices.items():
      if d.name == 'RC':
        print(k, d.name, adv)
        found = True
        break
    if not found:
      print('Not found')
"""
async def main():
  scanner = BleakScanner()
  while True:
    device = await scanner.find_device_by_name("nRF BLE")
    if device is None:
      raise Exception('Cannot find device')
    print(device, scanner.discovered_devices_and_advertisement_data.keys(), device.address)
"""

asyncio.run(main())
