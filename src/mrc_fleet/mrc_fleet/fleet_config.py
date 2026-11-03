"""Load the fleet definition (config/fleet.yaml) used by all launch files and nodes."""
import os
from dataclasses import dataclass
from typing import List

import yaml


@dataclass
class RobotSpec:
    name: str
    x: float
    y: float
    yaw: float

    @property
    def prefix(self) -> str:
        """TF frame prefix, e.g. 'robot1/'."""
        return f'{self.name}/'


def default_fleet_file() -> str:
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('mrc_fleet'), 'config', 'fleet.yaml')


def load_fleet(path: str = '') -> List[RobotSpec]:
    path = path or default_fleet_file()
    with open(path) as f:
        data = yaml.safe_load(f)
    robots = [RobotSpec(str(r['name']), float(r['x']), float(r['y']), float(r.get('yaw', 0.0)))
              for r in data['robots']]
    names = [r.name for r in robots]
    if len(set(names)) != len(names):
        raise ValueError(f'duplicate robot names in {path}: {names}')
    return robots
