#!/bin/zsh
set -u

duration="${1:-240}"
repo_root="$(cd "$(dirname "$0")/.." && pwd)"
log_dir="${repo_root}/../logs"
mkdir -p "${log_dir}"

stamp="$(date +"%Y-%m-%d_%H-%M-%S")"
start_time="$(date "+%Y-%m-%d %H:%M:%S")"
out="${log_dir}/orpheus_hardware_test_${stamp}.txt"
# The persisted replay (complete, no dropped lines) is also captured here so the
# soak summary can be parsed from it without double-counting the live stream.
replay_file="${log_dir}/.orpheus_replay_${stamp}.tmp"

predicate='(process == "kernel" AND (eventMessage CONTAINS[c] "ASFW" OR eventMessage CONTAINS[c] "[Audio]" OR eventMessage CONTAINS[c] "Orpheus" OR eventMessage CONTAINS[c] "Prism" OR eventMessage CONTAINS[c] "BeBoB" OR eventMessage CONTAINS[c] "AM824" OR eventMessage CONTAINS[c] "Isoch" OR eventMessage CONTAINS[c] "CMP" OR eventMessage CONTAINS[c] "PCR" OR eventMessage CONTAINS[c] "oPCR" OR eventMessage CONTAINS[c] "iPCR" OR eventMessage CONTAINS[c] "IRM claimed" OR eventMessage CONTAINS[c] "StartAllStreams" OR eventMessage CONTAINS[c] "AudioCoordinator" OR eventMessage CONTAINS[c] "AudioIOPath" OR eventMessage CONTAINS[c] "TxPipeline" OR eventMessage CONTAINS[c] "ASFWMIDI" OR eventMessage CONTAINS[c] "MIDI DIAGNOSTIC" OR eventMessage CONTAINS[c] "MIDI TX SELFTEST" OR eventMessage CONTAINS[c] "RX MIDI" OR eventMessage CONTAINS[c] "AVCAudioBackend" OR eventMessage CONTAINS[c] "BringUpPipeline" OR eventMessage CONTAINS[c] "Pipeline live" OR eventMessage CONTAINS[c] "initHardware" OR eventMessage CONTAINS[c] "CMD A" OR eventMessage CONTAINS[c] "CMDs B+C" OR eventMessage CONTAINS[c] "bringUpInput" OR eventMessage CONTAINS[c] "bringUpOutput" OR eventMessage CONTAINS[c] "Attach-time" OR eventMessage CONTAINS[c] "Bus role" OR eventMessage CONTAINS[c] "BusReset" OR eventMessage CONTAINS[c] "Discovery" OR eventMessage CONTAINS[c] "ConfigROM" OR eventMessage CONTAINS[c] "ROMReader" OR eventMessage CONTAINS[c] "ROMScanner" OR eventMessage CONTAINS[c] "SelfID" OR eventMessage CONTAINS[c] "Self-ID" OR eventMessage CONTAINS[c] "Topology" OR eventMessage CONTAINS[c] "FWDevice" OR eventMessage CONTAINS[c] "FWUnit" OR eventMessage CONTAINS[c] "GUID" OR eventMessage CONTAINS[c] "AR Request" OR eventMessage CONTAINS[c] "AR/RSP" OR eventMessage CONTAINS[c] "AsyncTransactionCompletion" OR eventMessage CONTAINS[c] "GetTransactionResult" OR eventMessage CONTAINS[c] "AsyncRead" OR eventMessage CONTAINS[c] "IN_TRANSITION" OR eventMessage CONTAINS[c] "FCPTransport")) OR (process == "ASFW" AND eventMessage CONTAINS[c] "Orpheus") OR (process == "coreaudiod" AND eventMessage CONTAINS[c] "ASFWAudioDevice")'
log_pid=""
interrupted=0

trap 'interrupted=1; if [[ -n "${log_pid}" ]]; then kill "${log_pid}" 2>/dev/null || true; fi' INT

{
  echo "ASFW Orpheus hardware test capture"
  echo "Started: ${start_time}"
  echo "Duration: ${duration}s"
  echo "Output: ${out}"
  echo
	  echo "Soak test: pass a longer duration in seconds, e.g."
	  echo "    $(basename "$0") 600     # 10-minute soak"
	  echo "Play continuous music for the whole window; a health summary is printed at the end."
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
  /usr/bin/log show --style compact --start "${start_time}" --predicate "${predicate}" 2>&1 \
    | tee "${replay_file}" || true
  echo

  echo "== DriverKit / FireWire / MIDI registry after log stream =="
  ioreg -r -l 2>&1 | /usr/bin/grep -E "ASFW|Orpheus|Prism|FireWire|IOFireWire|ASFWMIDI|MIDI I|MIDI O" || true
  echo

  echo "Finished: $(date)"
} | tee "${out}"

# ---------------------------------------------------------------------------
# Soak health summary. Parsed from the persisted replay (complete, no dropped
# lines, and not double-counted against the live stream). Surfaces exactly the
# signals that validate a clean run: SYT lock, TX underruns, HAL over/underruns,
# IT IRQ liveness, the zero-timestamp anchor bounds, and watchdog-backup kicks.
# ---------------------------------------------------------------------------
if [[ -s "${replay_file}" ]]; then
  src="${replay_file}"
else
  # Replay empty (e.g. interrupted before it ran): fall back to the combined log.
  src="${out}"
fi

{
  echo
  echo "== Soak health summary (auto-parsed) =="

  txcount="$(/usr/bin/grep -c 'TXRATE:' "${src}" 2>/dev/null || echo 0)"
  txfirst="$(/usr/bin/grep 'TXRATE:' "${src}" 2>/dev/null | head -1 | awk '{print $1" "$2}')"
  txlast="$(/usr/bin/grep 'TXRATE:' "${src}" 2>/dev/null | tail -1 | awk '{print $1" "$2}')"
  echo "TXRATE windows : ${txcount}   span ${txfirst:-none} -> ${txlast:-none}"

  echo -n "sytLead values : "
  /usr/bin/grep -oE 'sytLead=[-0-9]+' "${src}" 2>/dev/null | sort -u | tr '\n' ' '
  echo "  (want ONE value = SYT cycle-locked; multiple = drift/sawtooth)"

  urevents="$(/usr/bin/grep -c 'IT: UNDERRUN' "${src}" 2>/dev/null || echo 0)"
  utotal="$(/usr/bin/grep 'IT: UNDERRUN' "${src}" 2>/dev/null | grep -oE 'total=[0-9]+' | tail -1)"
  echo "IT underruns   : ${urevents} events, last ${utotal:-total=0}   (want ~0)"

  silmax="$(/usr/bin/grep -oE 'silPkt=[0-9]+' "${src}" 2>/dev/null | awk -F= 'BEGIN{m=0}{if($2>m)m=$2}END{print m}')"
  echo "silPkt max/win : ${silmax:-0}   (want 0)"

  echo -n "HAL over/under : "
  /usr/bin/grep -oE 'overruns=[0-9]+ underruns=[0-9]+/[0-9]+' "${src}" 2>/dev/null | sort -u | tail -4 | tr '\n' ' '
  echo "  (want overruns=0 underruns=0/0)"

  irqfirst="$(/usr/bin/grep -oE 'IRQ=[0-9]+' "${src}" 2>/dev/null | head -1)"
  irqlast="$(/usr/bin/grep -oE 'IRQ=[0-9]+' "${src}" 2>/dev/null | tail -1)"
  echo "IT IRQ counter : ${irqfirst:-none} -> ${irqlast:-none}   (must keep climbing, not freeze)"

  echo -n "anchor lead    : "
  /usr/bin/grep -oE 'lead=[-0-9]+' "${src}" 2>/dev/null | awk -F= 'BEGIN{n=0}{v=$2; if(n==0){mn=v;mx=v}else{if(v<mn)mn=v; if(v>mx)mx=v}; n++}END{if(n)printf "min %s / max %s (n=%d)\n", mn, mx, n; else print "none"}'
  echo -n "anchor n=      : "
  /usr/bin/grep -oE ' n=[0-9]+' "${src}" 2>/dev/null | sort -u | tr '\n' ' '
  echo "  (want n=1 = deadline-anchored rearm never skipped a period)"

  wd="$(/usr/bin/grep -oE 'wdKicks=[0-9]+' "${src}" 2>/dev/null | tail -1)"
  refillmax="$(/usr/bin/grep -oE 'maxRefillUs=[0-9]+' "${src}" 2>/dev/null | sort -t= -k2 -n | tail -1)"
  echo "watchdog       : ${wd:-wdKicks=?} ${refillmax:-maxRefillUs=?}   (kicks should stay low while IT IRQs are healthy)"
  echo
} | tee -a "${out}"

rm -f "${replay_file}"

echo
echo "Saved Orpheus hardware capture to: ${out}"
