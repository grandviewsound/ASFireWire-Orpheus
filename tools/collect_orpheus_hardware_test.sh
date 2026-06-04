#!/bin/zsh
set -u

duration="${1:-240}"
repo_root="$(cd "$(dirname "$0")/.." && pwd)"
log_dir="${repo_root}/../logs"
mkdir -p "${log_dir}"

stamp="$(date +"%Y-%m-%d_%H-%M-%S")"
start_time="$(date "+%Y-%m-%d %H:%M:%S")"
out="${log_dir}/orpheus_hardware_test_${stamp}.txt"

predicate='(process == "kernel" AND (eventMessage CONTAINS[c] "ASFW" OR eventMessage CONTAINS[c] "Orpheus" OR eventMessage CONTAINS[c] "Prism" OR eventMessage CONTAINS[c] "BeBoB" OR eventMessage CONTAINS[c] "AM824" OR eventMessage CONTAINS[c] "Isoch" OR eventMessage CONTAINS[c] "CMP" OR eventMessage CONTAINS[c] "PCR" OR eventMessage CONTAINS[c] "oPCR" OR eventMessage CONTAINS[c] "iPCR" OR eventMessage CONTAINS[c] "IRM claimed" OR eventMessage CONTAINS[c] "StartAllStreams" OR eventMessage CONTAINS[c] "AudioCoordinator" OR eventMessage CONTAINS[c] "AudioIOPath" OR eventMessage CONTAINS[c] "TxPipeline" OR eventMessage CONTAINS[c] "ASFWMIDI" OR eventMessage CONTAINS[c] "MIDI DIAGNOSTIC" OR eventMessage CONTAINS[c] "MIDI TX SELFTEST" OR eventMessage CONTAINS[c] "RX MIDI" OR eventMessage CONTAINS[c] "AVCAudioBackend" OR eventMessage CONTAINS[c] "BringUpPipeline" OR eventMessage CONTAINS[c] "Pipeline live" OR eventMessage CONTAINS[c] "initHardware" OR eventMessage CONTAINS[c] "CMD A" OR eventMessage CONTAINS[c] "CMDs B+C" OR eventMessage CONTAINS[c] "bringUpInput" OR eventMessage CONTAINS[c] "bringUpOutput" OR eventMessage CONTAINS[c] "Attach-time" OR eventMessage CONTAINS[c] "Bus role" OR eventMessage CONTAINS[c] "AR Request" OR eventMessage CONTAINS[c] "AR/RSP" OR eventMessage CONTAINS[c] "AsyncTransactionCompletion" OR eventMessage CONTAINS[c] "GetTransactionResult" OR eventMessage CONTAINS[c] "AsyncRead")) OR (process == "ASFW" AND eventMessage CONTAINS[c] "Orpheus") OR (process == "coreaudiod" AND eventMessage CONTAINS[c] "ASFWAudioDevice")'
log_pid=""
interrupted=0

trap 'interrupted=1; if [[ -n "${log_pid}" ]]; then kill "${log_pid}" 2>/dev/null || true; fi' INT

{
  echo "ASFW Orpheus hardware test capture"
  echo "Started: ${start_time}"
  echo "Duration: ${duration}s"
  echo "Output: ${out}"
  echo
	  echo "Suggested flow while this capture is running:"
	  echo "1. Open ASFW -> Orpheus, press Refresh, then enable Live meters."
	  echo "2. Keep the Orpheus page visible; use its Tone buttons after capture starts."
	  echo "3. Exercise the output pair(s) you want to test."
	  echo "4. Take a screenshot of the Orpheus page while the tone is playing."
	  echo "5. Press Control-C in this window if the useful part finishes early."
  echo

  echo "== DriverKit / FireWire / MIDI registry before log stream =="
  ioreg -r -l 2>&1 | /usr/bin/grep -E "ASFW|Orpheus|Prism|FireWire|IOFireWire|ASFWMIDI|MIDI I|MIDI O" || true
  echo

  echo "== FireWire system profile =="
  system_profiler SPFireWireDataType 2>&1 || true
  echo

  echo "== ASFW Orpheus log stream =="
  /usr/bin/log stream --style compact --level debug --predicate "${predicate}" 2>&1 &
  log_pid=$!
  sleep "${duration}"
  kill "${log_pid}" 2>/dev/null || true
  wait "${log_pid}" 2>/dev/null || true
  log_pid=""
  if [[ ${interrupted} -eq 1 ]]; then
    echo
    echo "Capture interrupted early; replaying persisted logs before exit."
  fi
  echo

  echo "== ASFW Orpheus persisted log replay =="
  /usr/bin/log show --style compact --start "${start_time}" --predicate "${predicate}" 2>&1 || true
  echo

  echo "== DriverKit / FireWire / MIDI registry after log stream =="
  ioreg -r -l 2>&1 | /usr/bin/grep -E "ASFW|Orpheus|Prism|FireWire|IOFireWire|ASFWMIDI|MIDI I|MIDI O" || true
  echo

  echo "Finished: $(date)"
} | tee "${out}"

echo
echo "Saved Orpheus hardware capture to: ${out}"
