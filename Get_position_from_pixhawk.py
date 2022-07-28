import asyncio
from mavsdk import System


async def run():
    # Init the drone
    drone = System()
    await drone.connect(system_address="serial:///dev/ttyPixhawk")

    asyncio.ensure_future(print_health(drone))
    asyncio.ensure_future(print_raw_gps_info(drone))

async def print_health(drone):
    async for health in drone.telemetry.health():
        print(f" Drone Health: {health}")

async def print_raw_gps_info(drone):
    async for raw_gps in drone.telemetry.raw_gps():
        #print(f" RAw GPS info: {raw_gps}")
        gps_time = raw_gps.timestamp_us
        print(f"GPS Time :  {gps_time}")

        gps_alt = raw_gps.absolute_altitude_m # absolute altitude of drone in meters
        print(f"GPS ALT :  {gps_alt}")

        gps_lat = raw_gps.latitude_deg #latitude of the gps
        print(f"GPS LAT :  {gps_lat}")

        gps_lng = raw_gps.longitude_deg #longitude of the gps
        print(f"GPS LNT :  {gps_lng}")
        print("ppppppppppppppppppppppppppppppppppppp")
if __name__ == "__main__":
    # Start the main function
    asyncio.ensure_future(run())

    # Runs the event loop until the program is canceled with e.g. CTRL-C
    asyncio.get_event_loop().run_forever()
