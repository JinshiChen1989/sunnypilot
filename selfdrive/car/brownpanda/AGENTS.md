# Repository Guidelines

## Project Structure & Module Organization
- Core code lives under `selfdrive/` (car interfaces, controls, monitoring) and `system/` (process manager, logging, camera, hardware).
- Shared utilities: `common/`; messaging schemas: `cereal/`.
- Vehicle data: `opendbc_repo/`, `opendbc/`; firmware/tools: `panda/`.
- Dev tools and replayers: `tools/`; vendored deps: `third_party/`.
- Entry points: `launch_openpilot.sh`, `SConstruct`, `pyproject.toml`.
- Example car module: `selfdrive/car/brownpanda/` with tests beside modules.

## Build, Test, and Development Commands
- Build natives: `scons -j$(nproc)` — compiles C/C++ components.
- Run tests: `pytest` — parallel by default; subset: `pytest path/to/test_file.py::test_name -m "not slow"`.
- Lint: `ruff check .`; format: `ruff format .`; types: `mypy .`.
- Run on device/sim: `./launch_openpilot.sh`.

## Coding Style & Naming Conventions
- Python 3.11, 2‑space indent, max line length 160.
- Prefer type hints; keep functions small and pure.
- Names: modules/functions `snake_case`, classes `PascalCase`, constants `UPPER_SNAKE_CASE`.
- Imports follow `openpilot.<pkg>` style per ruff banned‑imports config.

## Testing Guidelines
- Framework: `pytest` (+ xdist). Place tests next to modules under configured `testpaths`.
- Naming: files `test_*.py`, functions `test_*`.
- Marks: use `@pytest.mark.slow` or `@pytest.mark.tici` as needed; CI skips device‑specific tests by default.
- Coverage: add tests for new/changed logic; include realistic samples or replay routes for car logic.

## Commit & Pull Request Guidelines
- Commits: concise, imperative (e.g., `selfdrive: fix lateral tune`).
- PRs: clear description and rationale; link issues.
- Include reproduction steps, test plan (key `pytest` output), and logs/screenshots.
- For car changes: update DBC/fingerprints as needed; provide a short route segment for validation.

## Security & Configuration Tips
- Do not commit secrets or tokens; use env vars for local config.
- Verify changes locally with `scons` and `pytest` before opening a PR to keep CI green.
