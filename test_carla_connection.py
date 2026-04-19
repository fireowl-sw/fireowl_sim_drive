#!/usr/bin/env python3
"""
Simple CARLA connection test script
Connects to CARLA server and displays world information
"""

import carla
import time

def main():
    # Connect to CARLA server running on Windows (localhost:2000)
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    print("=" * 50)
    print("CARLA Connection Test")
    print("=" * 50)

    try:
        # Get the world
        world = client.get_world()
        print("✓ Connected to CARLA server!")

        # Display world info
        map_name = world.get_map().name
        print(f"\nMap: {map_name}")

        # Get weather info
        weather = world.get_weather()
        print(f"\nWeather:")
        print(f"  - Cloudiness: {weather.cloudiness}")
        print(f"  - Precipitation: {weather.precipitation}")
        print(f"  - Sun azimuth: {weather.sun_azimuth_angle}")
        print(f"  - Sun altitude: {weather.sun_altitude_angle}")

        # List all actors
        print("\n" + "-" * 50)
        print("Actors in the world:")
        print("-" * 50)
        actor_counts = {}
        for actor in world.get_actors():
            type_id = actor.type_id
            actor_counts[type_id] = actor_counts.get(type_id, 0) + 1

        for type_id, count in sorted(actor_counts.items()):
            print(f"  {type_id}: {count}")

        # List blueprints
        print("\n" + "-" * 50)
        print("Available Blueprints (first 20):")
        print("-" * 50)
        blueprint_library = world.get_blueprint_library()
        for i, bp in enumerate(blueprint_library):
            if i >= 20:
                break
            print(f"  {bp.id}")

        print(f"\n... and {len(blueprint_library) - 20} more blueprints")
        print("\n" + "=" * 50)
        print("Connection test completed successfully!")
        print("=" * 50)

    except Exception as e:
        print(f"\n✗ Error: {e}")
        print("\nMake sure CARLA is running on Windows with port 2000")
        print("You may need to allow WSL network access in Windows Firewall")

if __name__ == '__main__':
    main()
