from slam.sensors import LIDAR

def main():
    lidar = LIDAR("My LIDAR")
    lidar.print_info()

if __name__ == "__main__":
    main()
