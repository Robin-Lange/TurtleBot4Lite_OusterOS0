"""Fail closed on missing/empty/failed/skipped JUnit reports and write an Actions summary."""

import argparse
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


def summarize(path):
    root = ET.parse(path).getroot()
    suites = list(root.iter('testsuite'))
    # Parent suite aggregates must not double-count child suites.
    leaves = [suite for suite in suites if not suite.findall('testsuite')]
    if not leaves:
        raise ValueError('No test suites in report')
    counts = {key: sum(int(suite.get(key, 0)) for suite in leaves)
              for key in ('tests', 'failures', 'errors', 'skipped')}
    if any(value < 0 for value in counts.values()):
        raise ValueError('Negative test count')
    return counts


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('path', type=Path)
    parser.add_argument('--label', required=True)
    args = parser.parse_args()
    error = None
    try:
        counts = summarize(args.path)
        if counts['tests'] == 0 or any(counts[key] for key in ('failures', 'errors', 'skipped')):
            error = 'Expected executed tests with zero failures, errors, and skips'
        message = ', '.join(f'{value} {key}' for key, value in counts.items())
    except (OSError, ET.ParseError, ValueError) as exc:
        error = str(exc)
        message = f'Invalid or missing test report: {error}'
    print(f'{args.label}: {message}')
    summary = os.environ.get('GITHUB_STEP_SUMMARY')
    if summary:
        with open(summary, 'a', encoding='utf-8') as stream:
            stream.write(f'### {args.label}\n\n{message}\n\n')
    if error:
        print(error, file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
