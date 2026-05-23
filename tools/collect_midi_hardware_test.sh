#!/bin/zsh
set -u

duration="${1:-180}"
repo_root="$(cd "$(dirname "$0")/.." && pwd)"
log_dir="${repo_root}/../logs"
mkdir -p "${log_dir}"

stamp="$(date +"%Y-%m-%d_%H-%M-%S")"
out="${log_dir}/midi_hardware_test_${stamp}.txt"

predicate='eventMessage CONTAINS[c] "ASFWMIDI" OR eventMessage CONTAINS[c] "MIDI DIAGNOSTIC" OR eventMessage CONTAINS[c] "IR RX MIDI" OR eventMessage CONTAINS[c] "MIDI TX SELFTEST" OR eventMessage CONTAINS[c] "TX geometry" OR eventMessage CONTAINS[c] "s12=0x"'

{
  echo "ASFW MIDI hardware test capture"
  echo "Started: $(date)"
  echo "Duration: ${duration}s"
  echo "Output: ${out}"
  echo

  echo "== DriverKit services before log stream =="
  ioreg -r -l -c ASFWMIDINub 2>&1 || true
  echo
  ioreg -r -l -c ASFWMIDIDriver 2>&1 || true
  echo

  echo "== CoreMIDI/MIDI-related IORegistry names =="
  ioreg -r -l 2>&1 | /usr/bin/grep -E "ASFWMIDI|MIDI I 1|MIDI O|ASFW-MIDI|Orpheus" || true
  echo

  echo "== FireWire system profile =="
  system_profiler SPFireWireDataType 2>&1 || true
  echo

  echo "== ASFW MIDI log stream =="
  echo "Unplug/replug or start the driver path now if needed, then let tone playback run."
  /usr/bin/log stream --style compact --level info --predicate "${predicate}" 2>&1 &
  log_pid=$!
  sleep "${duration}"
  kill "${log_pid}" 2>/dev/null || true
  wait "${log_pid}" 2>/dev/null || true
  echo

  echo "== DriverKit services after log stream =="
  ioreg -r -l -c ASFWMIDINub 2>&1 || true
  echo
  ioreg -r -l -c ASFWMIDIDriver 2>&1 || true
  echo

  echo "Finished: $(date)"
} | tee "${out}"

echo
echo "Saved MIDI hardware capture to: ${out}"
