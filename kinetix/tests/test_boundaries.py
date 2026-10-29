"""Who may import whom (architecture.md §4, collaboration.md): the two areas meet only in kinetix/ and the run directory."""
import ast

from kinetix.config import ROOT

RULES = {                                          # folder: imports it must never make
    "kinetix": ("acquisition", "reconstruction", "rclpy"),
    "reconstruction": ("acquisition", "rclpy"),
    "acquisition/planner": ("rclpy", "acquisition.rig.mavros", "acquisition.rig.dry", "reconstruction"),
    "acquisition/mapping": ("rclpy", "acquisition.rig.mavros", "acquisition.rig.dry", "reconstruction"),
}
ENTRY = {ROOT / "kinetix" / "cli.py"}              # the CLI dispatches to both areas


def _imports(f):
    for node in ast.walk(ast.parse(f.read_text())):
        if isinstance(node, ast.Import):
            yield from (a.name for a in node.names)
        elif isinstance(node, ast.ImportFrom) and node.level == 0:
            yield node.module or ""


def test_import_boundaries():
    for folder, bad in RULES.items():
        files = [f for f in (ROOT / folder).rglob("*.py") if f not in ENTRY and "tests" not in f.parts]
        assert files, f"{folder}: no sources found (moved?)"
        for f in files:
            for n in _imports(f):
                assert not any(n == b or n.startswith(b + ".") for b in bad), f"{f.relative_to(ROOT)}: imports {n}"
