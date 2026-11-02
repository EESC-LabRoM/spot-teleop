"""Load the local handover policy without shadowing the installed cuRobo package."""
from pathlib import Path
from runpy import run_path
_policy = run_path(str(Path(__file__).with_name('curobo') / 'curobo_start.py'))
prepare = _policy['prepare']
start_reason = _policy['start_reason']
