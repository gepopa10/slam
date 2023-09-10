from slam.sensors import LIDAR

def test_lidar_name():
    lidar = LIDAR("Test LIDAR")
    assert lidar.name == "Test LIDAR"
