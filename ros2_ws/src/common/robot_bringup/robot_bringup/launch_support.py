"""Start the mission only after a successful, stable readiness check."""
from launch.actions import EmitEvent, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from pathlib import Path


def prepare_run_directory(value):
    path = Path(value).expanduser().resolve()
    path.mkdir(parents=True, exist_ok=True)
    try:
        with (path/'.nav-vi-run').open('x', encoding='utf-8') as stream:
            stream.write('One run per directory. Use a new output_dir for another run.\n')
    except FileExistsError as exc:
        raise ValueError(f'Run directory already used: {path}; choose a new output_dir') from exc
    return path


def after_preflight(check, mission):
    def finished(event, context):
        if event.returncode == 0:
            return [LogInfo(msg='NAV/VI preflight passed; starting mission'), mission]
        return [LogInfo(msg='NAV/VI preflight failed; mission was not started'),
                EmitEvent(event=Shutdown(reason='Readiness check failed'))]
    return RegisterEventHandler(OnProcessExit(target_action=check, on_exit=finished))
