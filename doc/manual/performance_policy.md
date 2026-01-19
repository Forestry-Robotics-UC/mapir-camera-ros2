# Performance Regression Policy

Performance tests run in CI to detect regressions in core processing paths.
Each test measures a representative workload on a fixed input size and asserts
that runtime stays below a conservative threshold.

## Thresholds

Default per-test limits (can be overridden by environment variables in CI):

- `MAPIR_PERF_MAX_MS_CORE`: 200 ms per test case

These thresholds are intentionally conservative to reduce flakiness. If a
regression is detected, the change must either:

- improve performance or restore the previous timing, or
- update the threshold with justification in the pull request and quality
  declaration notes.

## Running Locally

```
pytest -m performance
```
