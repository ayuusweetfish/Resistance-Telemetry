import asyncio
import bleak
from bleak import BleakScanner, BleakClient
import datetime

async def main():
  devices = await BleakScanner.discover(return_adv=True)
  for k, (d, adv) in devices.items():
    print(k, d.name, adv)

asyncio.run(main())
