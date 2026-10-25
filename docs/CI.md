# GitHub Actions checks

The `CI` workflow tests the supported ROS 2 Humble / Ubuntu 22.04 software stack. It never connects to the lab network, starts a hardware action, or deploys to a robot.

## Triggers and status checks

- Pushes to `main` and `feature/realsense-rplidar-navigation` test the committed integration branches.
- Every pull request is tested, including draft and fork PRs, using the normal `pull_request` event and its merge result. Feature pushes rely on their PR check instead of launching an identical push run.
- `workflow_dispatch` supports manual runs for a selected ref. GitHub must know the workflow on the default branch for its manual-run UI.
- New commits cancel obsolete runs for the same event/ref or PR. Push and PR merge checks have separate concurrency groups.
- No path filter skips docs-only changes; a required check should always reach a result.

| Check | Validation |
| --- | --- |
| Code Quality and Linting | Upstream actionlint, ShellCheck, Flake8, Python compilation, pure numerical systems, configuration contracts, map serialization |
| ROS 2 Humble Build & Test (amd64) | Strict rosdep installation, colcon build, all ROS adapter and integration tests on an x86-64 runner |
| ROS 2 Humble Build & Test (arm64) | The same build and tests on a native ARM64 runner, matching the Jetson CPU architecture |
| CI required | Fails unless quality and both ROS matrix jobs succeed; fails if an upstream job was skipped or cancelled |

ROS matrix jobs start after quality passes, run independently, and use `fail-fast: false` so a failure on one architecture does not hide the result of the other. Timeouts are 10 minutes for quality and 20 minutes per ROS build. Tests use localhost-only discovery on domain 88.

Use the stable **CI required** status as the required check in a branch ruleset, with pull requests and review required for protected integration branches. Workflow YAML defines the check but does not enable repository branch protection. No repository rules were changed by this patch. If merge queue is adopted later, add its `merge_group` trigger before requiring this workflow there.

## Test results and failure diagnosis

The workflow writes test counts to the Actions job summary and retains artifacts for 14 days:

- `pure-test-results`: pure-test JUnit XML.
- `ros-humble-results-amd64` and `ros-humble-results-arm64`: JUnit XML, apt/rosdep logs, colcon logs, and installed package/architecture information.

`report_tests.py` fails on missing or malformed reports, zero executed tests, test failures, errors, or skips. Pure CI selects the three ROS-independent test modules explicitly; ROS CI collects the entire package and requires zero skipped tests. Artifacts and summaries are attempted even after a failed build or test step. Uploading artifacts does not turn a failed test into a passing check.

## Dependency and token handling

Actions are pinned to verified full commit SHAs with release comments. Current pins use Node 24 actions. The workflow token has only `contents: read`, checkout does not persist credentials, and no secrets are passed to tests. The workflow uses `pull_request`, not a privileged `pull_request_target` execution of contributed code.

CI Python tools are version-pinned in `.github/requirements-ci.txt`. `setup-python` caches pip downloads using that file as the key. ROS apt binaries, workspace builds, and installations are not reused from a cache: each architecture resolves the package manifest afresh. The image tag fixes the ROS distribution and Ubuntu release; apt packages and the image can still receive updates. Saved environment metadata identifies the versions actually tested.

The proposed `.github/dependabot.yml` groups weekly action and CI-tool update PRs targeting the development branch. **Dependabot reads its configuration from the default branch (`main`); updates do not become active merely by merging this file into the development branch.** Keep the version comments next to SHA pins so action update PRs remain readable. The actionlint version and digest in `scripts/ci/check_workflows.sh` must be updated together from the upstream release.

## Run checks locally

From the repository root, in a separate Python environment:

```bash
python3 -m pip install -r .github/requirements-ci.txt
bash scripts/ci/check_workflows.sh
shellcheck --severity=warning --exclude=SC1090,SC1091 setup_env.sh scripts/*.sh scripts/ci/*.sh
python3 -m flake8 --config .flake8 ros2_ws/src/ scripts/ci/
```

The actionlint download helper supports Linux x86-64. On another architecture, install native upstream actionlint and set `ACTIONLINT_BIN=/path/to/actionlint` before running the helper. SC1090/SC1091 are excluded for sourced ROS environments whose files and paths are specific to the target machine.

## Scope of validation

Native ARM64 CI checks package availability, Python/native message compatibility, launch construction, and runtime behavior without hardware. It does not validate JetPack, a Jetson kernel, RealSense firmware or USB drivers, GPU operation, sensor calibration, battery capacity, or braking. Those remain on-robot commissioning checks. Do not attach an automatically triggered public-PR runner to a powered robot.

Primary references:

- [GitHub Actions secure use](https://docs.github.com/en/actions/reference/security/secure-use)
- [Workflow concurrency](https://docs.github.com/en/actions/how-tos/write-workflows/choose-when-workflows-run/control-workflow-concurrency)
- [GitHub-hosted runner architectures](https://docs.github.com/en/actions/reference/runners/github-hosted-runners)
- [Dependency caching](https://docs.github.com/en/actions/reference/workflows-and-actions/dependency-caching)
- [Dependabot configuration location](https://docs.github.com/en/code-security/concepts/supply-chain-security/about-the-dependabot-yml-file)
- [actionlint](https://github.com/rhysd/actionlint)
