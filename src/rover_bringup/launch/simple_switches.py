"""Translate user-facing yes/no switches to the existing bringup arguments."""


def launch_arguments(values):
    def parse(name):
        value = values[name].strip().lower()
        if value in ('yes', 'true', '1'):
            return True
        if value in ('no', 'false', '0'):
            return False
        raise ValueError(f"{name} must be yes or no (received {values[name]!r})")

    selected = {name: parse(name) for name in
                ('zed', 'icm', 'gps', 'wheel', 'localization', 'drivers')}
    enabled = {
        'enable_zed': selected['drivers'] and selected['zed'],
        'enable_imu': selected['drivers'] and selected['icm'],
        'enable_gps': selected['drivers'] and selected['gps'],
        'enable_localization': selected['localization'],
        'enable_global': selected['gps'] and selected['localization'],
        'use_zed_odom': selected['zed'],
        'use_zed_imu': selected['zed'],
        'use_icm_imu': selected['icm'],
        'use_wheel_odom': selected['wheel'],
    }
    return {key: str(value).lower() for key, value in enabled.items()}
