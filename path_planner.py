from dataclasses import dataclass


@dataclass
class PathPosition:
    x: float
    y: float
    yaw: float = None
    speed: float = None
    allow_overshoot: bool = None


class PathPlanner:
    def __init__(self, positions=None):
        self._positions = []

        if positions is not None:
            self.set_path(positions)

    def set_path(self, positions):
        self.clear()

        for position in positions:
            self.add_position(position)

    def add_position(self, position, y=None, yaw=None, speed=None, allow_overshoot=None):
        if isinstance(position, PathPosition):
            self._positions.append(position)
            return

        if isinstance(position, dict):
            self._positions.append(
                PathPosition(
                    x=float(position["x"]),
                    y=float(position["y"]),
                    yaw=position.get("yaw"),
                    speed=position.get("speed"),
                    allow_overshoot=position.get("allow_overshoot"),
                )
            )
            return

        if y is None:
            x, y = position[0], position[1]
            if len(position) > 2:
                yaw = position[2]
            if len(position) > 3:
                speed = position[3]
            if len(position) > 4:
                allow_overshoot = position[4]
        else:
            x = position

        self._positions.append(
            PathPosition(
                x=float(x),
                y=float(y),
                yaw=yaw,
                speed=speed,
                allow_overshoot=allow_overshoot,
            )
        )

    def clear(self):
        self._positions = []

    def path(self):
        return list(self._positions)

    def position(self, index):
        return self._positions[index]

    def is_empty(self):
        return len(self._positions) == 0

    def __len__(self):
        return len(self._positions)
