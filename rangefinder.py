class RangefinderReader:
    sensor_names = {
        "front_right": "rangefinder_front_right",
        "front_left": "rangefinder_front_left",
        "rear_right": "rangefinder_rear_right",
        "rear_left": "rangefinder_rear_left",
        "rear": "rangefinder_rear",
        "front": "rangefinder_front",
    }

    def __init__(self, model, data):
        self.model = model
        self.data = data
        self.sensor_names = self.sensor_names
        self.sensor_addresses = {
            name: self._sensor_address(sensor_name)
            for name, sensor_name in self.sensor_names.items()
        }

    def distance(self, name):
        return float(self.data.sensordata[self.sensor_addresses[name]])

    def distances(self):
        return {
            name: self.distance(name)
            for name in self.sensor_names
        }

    def detected_distances(self, no_detection=None):
        return {
            name: distance if distance >= 0.0 else no_detection
            for name, distance in self.distances().items()
        }

    def _sensor_address(self, sensor_name):
        sensor_id = self.model.sensor(sensor_name).id
        sensor_dim = int(self.model.sensor_dim[sensor_id])
        if sensor_dim != 1:
            raise ValueError(
                f"Expected rangefinder sensor '{sensor_name}' to have dimension 1, "
                f"got {sensor_dim}"
            )
        return int(self.model.sensor_adr[sensor_id])
