#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
tools/.metrics-venv/bin/python tools/ros1_metrics/collect.py
tools/.metrics-venv/bin/python tools/ros1_metrics/collect_oo.py
tools/.metrics-venv/bin/python tools/ros1_metrics/render_report.py
