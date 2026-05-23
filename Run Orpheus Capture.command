#!/bin/zsh
set -u

script_dir="${0:A:h}"
repo_dir="${script_dir}"
if [[ ! -x "${repo_dir}/tools/collect_orpheus_hardware_test.sh" ]]; then
  repo_dir="/Users/kevinpeters/Desktop/Projects/Claude Projects/Firewire/ASFireWire-Orpheus"
fi

capture_script="${repo_dir}/tools/collect_orpheus_hardware_test.sh"
log_dir="${repo_dir}/../logs"
log_dir="${log_dir:A}"
default_duration=180

clear
echo "ASFW Orpheus Capture"
echo
echo "This will collect one hardware-test log and save it to:"
echo "${log_dir}"
echo
echo "Before starting:"
echo "  1. Build/relaunch the ASFW app if you just changed it."
echo "  2. Open ASFW -> Orpheus."
echo "  3. Press Refresh, then enable Live meters."
echo "  4. Stay on the Orpheus page; use its Tone buttons after capture starts."
echo
echo -n "Capture duration in seconds [${default_duration}]: "
read duration

if [[ -z "${duration}" ]]; then
  duration="${default_duration}"
fi

if ! [[ "${duration}" == <-> ]]; then
  echo
  echo "Duration must be a whole number of seconds."
  echo "Press Return to close."
  read _
  exit 1
fi

if [[ ! -x "${capture_script}" ]]; then
  echo
  echo "Capture script is not executable:"
  echo "${capture_script}"
  echo
  echo "Press Return to close."
  read _
  exit 1
fi

echo
echo "When you press Return, start a Tone button on the Orpheus page as soon as the log stream begins."
echo "Capture will run for ${duration} seconds."
echo
echo "Press Return to start."
read _

"${capture_script}" "${duration}"
status=$?

echo
if [[ ${status} -eq 0 ]]; then
  echo "Capture finished."
  /usr/bin/open "${log_dir}" >/dev/null 2>&1 || true
else
  echo "Capture exited with status ${status}."
fi
echo
echo "Press Return to close this window."
read _
exit ${status}
