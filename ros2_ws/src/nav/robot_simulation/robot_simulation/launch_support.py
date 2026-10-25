"""Reject the default robot domain before launching any simulation processes."""
from launch.substitutions import LaunchConfiguration


def validate_domain(context):
    domain = int(LaunchConfiguration('domain_id').perform(context))
    if not 1 <= domain <= 232:
        raise ValueError('Simulation domain_id must be between 1 and 232; default is 75')
    return []
