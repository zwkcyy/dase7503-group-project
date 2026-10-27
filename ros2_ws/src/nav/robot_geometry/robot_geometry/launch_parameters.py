"""Expand launch expressions before Nav2 rewrites YAML string escaping."""
from launch import Substitution
from launch_ros.descriptions import ParameterFile


class ExpandedParameterFile(Substitution):
    """Resolve the original template once, before RewrittenYaml serializes it."""
    def __init__(self, source):
        super().__init__()
        self.source = ParameterFile(source, allow_substs=True)

    def perform(self, context):
        return str(self.source.evaluate(context))

    def describe(self):
        return 'Parameter template expanded before YAML rewriting'
