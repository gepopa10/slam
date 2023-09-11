import pytest
import math

from slam.sensors import laser_sensor

def mocked_get_size():
    return (800, 600)

def mocked_get_surface():
    class MockSurface:
        @staticmethod
        def get_size():
            return mocked_get_size()
    return MockSurface()

@pytest.fixture
def mock_pygame(monkeypatch):
    monkeypatch.setattr("pygame.display.get_surface", mocked_get_surface)
    return True  # This is optional. You can return anything or nothing.


@pytest.fixture
def sensor(mock_pygame):
    return laser_sensor(10,10,[10,10])

def test_lidar_name(sensor):  # Use the fixture as an argument
    assert sensor.distance([2,2]) == math.sqrt(8)
