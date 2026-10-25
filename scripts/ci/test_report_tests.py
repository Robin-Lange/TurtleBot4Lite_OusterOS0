"""Guard the CI gate against a false green on missing, empty, or failed test output."""

import os
from pathlib import Path
import subprocess
import sys

import pytest


@pytest.mark.parametrize('xml, expected', [
    ('<testsuite tests="1" failures="0" errors="0" skipped="0"/>', 0),
    ('<testsuites><testsuite tests="1"/></testsuites>', 0),
    ('<testsuite tests="2"><testsuite tests="2"/></testsuite>', 0),
    ('<testsuite tests="1" failures="1"/>', 1),
    ('<testsuite tests="1" errors="1"/>', 1),
    ('<testsuite tests="1" skipped="1"/>', 1),
    ('<testsuite tests="0"/>', 1),
    ('<testsuite tests="-1"/>', 1),
    ('<broken', 1),
    (None, 1),
])
def test_result_verifier_exit_status_and_summary(tmp_path, xml, expected):
    report = tmp_path/'report.xml'
    if xml is not None:
        report.write_text(xml)
    summary = tmp_path/'summary.md'
    env = dict(os.environ, GITHUB_STEP_SUMMARY=str(summary))
    result = subprocess.run(
        [sys.executable, str(Path(__file__).with_name('report_tests.py')), str(report), '--label', 'Guard test'],
        capture_output=True, text=True, env=env, timeout=10,
    )
    assert result.returncode == expected, result.stdout + result.stderr
    assert 'Guard test' in summary.read_text()
    if xml and '<testsuite tests="2"><testsuite' in xml:
        assert '2 tests' in result.stdout  # Do not double-count aggregate parent suites.
