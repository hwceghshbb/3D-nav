"""Select a Python interpreter that can actually run a SONIC subprocess."""

from __future__ import annotations

import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from typing import Iterable, Sequence


_PROBE_TIMEOUT_SECONDS = 15.0
_SELECTED_PYTHON_ENV = "BXI_SONIC_SELECTED_PYTHON"
_SCRIPT_BOOTSTRAP = (
    "import os, runpy, sys\n"
    "script = os.path.abspath(sys.argv[1])\n"
    "sys.argv = sys.argv[1:]\n"
    "script_dir = os.path.dirname(script)\n"
    "sys.path.insert(0, script_dir)\n"
    "runpy.run_path(script, run_name='__main__')\n"
)


def _clean_python_environment() -> dict[str, str]:
    """Remove every parent-Python control variable from the vendor process."""
    environment = os.environ.copy()
    for name in tuple(environment):
        if name.startswith("PYTHON") or name == "__PYVENV_LAUNCHER__":
            environment.pop(name, None)
    return environment


def _resolve_python(value: str) -> Path | None:
    value = value.strip()
    if not value:
        return None
    if os.sep not in value:
        resolved = shutil.which(value)
        return Path(resolved).resolve() if resolved else None
    candidate = Path(value).expanduser()
    if not candidate.is_absolute():
        candidate = Path.cwd() / candidate
    try:
        candidate = candidate.resolve()
    except OSError:
        return None
    return candidate if candidate.is_file() and os.access(candidate, os.X_OK) else None


def _unique_paths(values: Iterable[str | Path]) -> tuple[Path, ...]:
    result: list[Path] = []
    seen: set[Path] = set()
    for value in values:
        candidate = _resolve_python(str(value))
        if candidate is None or candidate in seen:
            continue
        seen.add(candidate)
        result.append(candidate)
    return tuple(result)


def _common_candidates() -> tuple[Path, ...]:
    home = Path.home()
    values: list[str | Path] = [sys.executable]
    for variable in ("VIRTUAL_ENV", "CONDA_PREFIX"):
        root = os.environ.get(variable)
        if root:
            values.append(Path(root) / "bin" / "python")
    values.extend(
        (
            home / "miniconda3" / "bin" / "python",
            home / "anaconda3" / "bin" / "python",
            "python3",
            "python",
        )
    )
    return _unique_paths(values)


def _conda_candidates() -> tuple[Path, ...]:
    conda = shutil.which("conda")
    if conda is None:
        for candidate in (
            Path.home() / "miniconda3" / "bin" / "conda",
            Path.home() / "anaconda3" / "bin" / "conda",
        ):
            if candidate.is_file() and os.access(candidate, os.X_OK):
                conda = str(candidate)
                break
    if conda is None:
        return ()
    try:
        completed = subprocess.run(
            (conda, "env", "list", "--json"),
            check=True,
            capture_output=True,
            text=True,
            timeout=10.0,
        )
        roots = json.loads(completed.stdout).get("envs", ())
    except (OSError, subprocess.SubprocessError, ValueError, AttributeError):
        return ()
    if not isinstance(roots, list):
        return ()
    return _unique_paths(Path(root) / "bin" / "python" for root in roots)


def _probe(
    interpreter: Path,
    imports: Sequence[str],
) -> tuple[bool, str]:
    code = (
        "import importlib\n"
        f"names = {tuple(imports)!r}\n"
        "for name in names:\n"
        "    importlib.import_module(name)\n"
    )
    try:
        completed = subprocess.run(
            (str(interpreter), "-E", "-s", "-c", code),
            check=False,
            capture_output=True,
            text=True,
            timeout=_PROBE_TIMEOUT_SECONDS,
            env=_clean_python_environment(),
        )
    except subprocess.TimeoutExpired:
        return False, f"dependency probe timed out after {_PROBE_TIMEOUT_SECONDS:.0f}s"
    except OSError as exc:
        return False, str(exc)
    if completed.returncode == 0:
        return True, ""
    output = (completed.stderr or completed.stdout).strip().splitlines()
    return False, output[-1] if output else f"probe exited {completed.returncode}"


def select_python(
    component: str,
    imports: Sequence[str],
) -> Path:
    """Return an interpreter satisfying imports, or raise one complete error."""
    explicit = os.environ.get("SONIC_PICO_PYTHON", "").strip()
    if explicit:
        candidate = _resolve_python(explicit)
        if candidate is None:
            raise RuntimeError(
                f"SONIC_PICO_PYTHON is not an executable Python: {explicit}"
            )
        candidates = (candidate,)
    else:
        candidates = _common_candidates()

    failures: list[str] = []
    checked_candidates: set[Path] = set()

    def try_candidates(values: Sequence[Path]) -> Path | None:
        for candidate in values:
            if candidate in checked_candidates:
                continue
            checked_candidates.add(candidate)
            available, reason = _probe(candidate, imports)
            if available:
                print(
                    f"[sonic-python] {component}: selected {candidate}",
                    flush=True,
                )
                return candidate
            failures.append(f"  - {candidate}: {reason}")
        return None

    selected = try_candidates(candidates)
    if selected is not None:
        return selected
    if not explicit:
        selected = try_candidates(_conda_candidates())
        if selected is not None:
            return selected

    checked = "\n".join(failures) if failures else "  - no Python candidates found"
    requirements = Path(__file__).resolve().parents[1] / "requirements-pico.txt"
    raise RuntimeError(
        f"no Python interpreter can run SONIC {component}; required imports: "
        f"{', '.join(imports)}\nchecked:\n{checked}\n"
        "Install the dependencies into one environment:\n"
        f"  <python> -m pip install -r {requirements}\n"
        "Install the matching xrobotoolkit_sdk wheel when required, then either "
        "set SONIC_PICO_PYTHON=<python> or restart the Mod for auto-discovery."
    )


def reexec_if_needed(component: str, imports: Sequence[str]) -> None:
    try:
        current = Path(sys.executable).resolve()
    except OSError:
        current = Path(sys.executable)
    previously_selected = _resolve_python(os.environ.get(_SELECTED_PYTHON_ENV, ""))
    if previously_selected == current:
        return
    selected = select_python(component, imports)
    environment = _clean_python_environment()
    environment[_SELECTED_PYTHON_ENV] = str(selected)
    os.execve(
        str(selected),
        (
            str(selected),
            "-E",
            "-s",
            "-c",
            _SCRIPT_BOOTSTRAP,
            str(Path(sys.argv[0]).resolve()),
            *sys.argv[1:],
        ),
        environment,
    )


__all__ = ["reexec_if_needed", "select_python"]
