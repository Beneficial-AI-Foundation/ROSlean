"""Upstream's nav2_collision_monitor with the scenarios of probes.py added.

    scenario.py --profile profile_upstream          # scenario.py's and the probes
    scenario.py --profile profile_upstream --own    # the probes only

Nothing is expected to differ: the expectations of both files are what
upstream's binary does.
"""

from probes import PROBES

STATE_TOPIC = True
TF = True
SCENARIOS = PROBES
EXPECT = {}
