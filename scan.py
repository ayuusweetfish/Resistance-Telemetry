import asyncio
import bleak
from bleak import BleakScanner, BleakClient
import datetime

async def main():
  while True:
    devices = await BleakScanner.discover(timeout=0.5, return_adv=True)
    for k, (d, adv) in devices.items():
      if d.name == 'nRF BLE':
        print(k, d.name, adv)
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
