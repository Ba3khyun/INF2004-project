#!/bin/bash
# Build and run Buddy 1's PC unit tests (Linux / WSL / macOS, needs gcc).
#   ./run_host_tests.sh      -> [TEST] lines; exit status 0 only if all pass
# Then runs GCC's static analyser (-fanalyzer) on the production modules.
cd "$(dirname "$0")/../../.." || exit 1		# repository root
APP=app_program
OUT="${TMPDIR:-/tmp}/s1_host_tests"
WARN=(-std=c11 -Wall -Wextra -Werror -Wpedantic -Wconversion -Wno-sign-conversion
      -Wshadow -Wcast-align)
INC=(-I"$APP" -I"$APP/subsystem1")
SRC=(testing/subsystem1/host/test_wifi_host.c "$APP/subsystem1/wifi_cmd.c"
     "$APP/subsystem1/wifi_sm.c")

gcc "${WARN[@]}" -O1 -g -fsanitize=address,undefined -fno-sanitize-recover=all \
    "${INC[@]}" "${SRC[@]}" -o "$OUT" || exit 2
"$OUT"
rc=$?

for f in "$APP/subsystem1/wifi_cmd.c" "$APP/subsystem1/wifi_sm.c"; do
	gcc -std=c11 -fanalyzer -Wall -Wextra -Werror "${INC[@]}" -c "$f" -o /dev/null || rc=3
done
echo "static analysis (gcc -fanalyzer -Werror): $([ $rc -ne 3 ] && echo clean || echo FINDINGS)"
exit $rc
