
import asyncio
from bleak import BleakScanner

async def main():
    print("Scanning for 10 seconds...\n")

    devices = await BleakScanner.discover(
        timeout=10,
        return_adv=True
    )

    for device, adv in devices.values():
        print("-----------------------------")
        print("Name:", device.name)
        print("Local name:", adv.local_name)
        print("Address:", device.address)
        print("Service UUIDs:", adv.service_uuids)
        print("RSSI:", adv.rssi)

asyncio.run(main())
