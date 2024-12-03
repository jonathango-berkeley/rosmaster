import argparse
import yaml
from math import pi
from pathlib import Path

def generate_yaml_config(file_name: str):
    # Ensure the file name has the ".yaml" extension
    if not file_name.endswith(".yaml"):
        file_name += ".yaml"

    # Path to the "config" directory
    config_folder = Path(__file__).parent.parent / "config"
    config_folder.mkdir(parents=True, exist_ok=True)  # Create the directory if it doesn't exist

    # Full path for the configuration file
    full_path = config_folder / file_name

    # Check if the file already exists
    if full_path.exists():
        confirm = input(f"File '{full_path}' already exists. Overwrite? (type 'y' to confirm): ").strip().lower()
        if confirm != "y":
            print("Operation cancelled. No changes were made.")
            return

    # Default configuration values
    config = {
        "lidar": {
            "topic": "/scan",               # Topic for LiDAR data
            "frame_id": "laser_frame",      # Frame of reference for the LiDAR
            "range_min": 0.12,              # Minimum range of the LiDAR (in meters)
            "range_max": 10.0,              # Maximum range of the LiDAR (in meters)
            "angle_min": -pi,               # Starting angle of the LiDAR scan (in radians)
            "angle_max": pi,                # Ending angle of the LiDAR scan (in radians)
            "angle_increment": 0.0174533,   # Angular resolution of the LiDAR (in radians)
        },
        "occupancy_grid": {
            "topic": "/map",                # Topic for OccupancyGrid
            "map_width": 100,               # Width of the map grid (in cells)
            "map_height": 100,              # Height of the map grid (in cells)
            "resolution": 0.1,              # Grid cell resolution (in meters per cell)
            "origin": {
                "x": 0.0,                  # X-coordinate of the map origin (in meters)
                "y": 0.0,                  # Y-coordinate of the map origin (in meters)
                "z": 0.0,                   # Z-coordinate of the map origin (in meters)
            },
            "unknown_value": -1,            # Value for unknown cells
            "occupied_value": 100,          # Value for occupied cells
            "free_value": 0,                # Value for free cells
        }
    }

    # Write the configuration to a YAML file
    with full_path.open('w') as yaml_file:
        yaml.dump(config, yaml_file, default_flow_style=False)

    print(f"YAML configuration file '{full_path}' created successfully!")


def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Generate a YAML configuration file for LiDAR and OccupancyGrid.")
    parser.add_argument("name", type=str, help="Name of the YAML configuration file (without the '.yaml' extension).")

    args = parser.parse_args()

    # Generate the YAML configuration file
    generate_yaml_config(args.name)


if __name__ == "__main__":
    main()
