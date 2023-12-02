import asyncio
import bleak
from bleak import BleakScanner, BleakClient
import datetime

async def main():
  devices = await BleakScanner.discover(return_adv=True)
  for k, (d, adv) in devices.items():
    print(k, d.name, adv)
    if d.name == 'nRF BLE':
      print(d)
"""
async def main():
  device = await BleakScanner.find_device_by_name("nRF BLE")
  if device is None:
    raise Exception('Cannot find device')
  print(device)
  async with BleakClient(device) as client:
    def notify_callback(sender: bleak.BleakGATTCharacteristic, data: bytearray):
      # print(data)
      value = (data[0] << 16) | (data[1] << 8) | data[2]
      print(datetime.datetime.now(), hex(value))
    await client.start_notify(
      '0000181c-0000-1000-8000-00805f9b34fb',
      notify_callback
    )
    await asyncio.sleep(100)
"""

asyncio.run(main())
