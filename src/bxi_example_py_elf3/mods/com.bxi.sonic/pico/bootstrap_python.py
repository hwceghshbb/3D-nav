"""Recover from a parent process that polluted Python's startup environment."""

import os
import sys


_BOOTSTRAPPED_ENV = "BXI_SONIC_BOOTSTRAPPED"


def ensure_clean_start() -> None:
    """Re-exec this script once before importing non-trivial stdlib modules."""
    if os.environ.get(_BOOTSTRAPPED_ENV) == "1":
        return
    environment = os.environ.copy()
    for name in tuple(environment):
        if name.startswith("PYTHON") or name == "__PYVENV_LAUNCHER__":
            environment.pop(name, None)
    environment[_BOOTSTRAPPED_ENV] = "1"
    os.execve(
        sys.executable,
        (
            sys.executable,
            "-E",
            "-s",
            os.path.abspath(sys.argv[0]),
            *sys.argv[1:],
        ),
        environment,
    )


__all__ = ["ensure_clean_start"]
