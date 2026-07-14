#!/usr/bin/env bash
#
# led.sh -- manually control the Crazyflie bottom Color LED deck (colorLedBot).
#
# Maps color NAMES to the firmware param colorLedBot.wrgb8888 (uint32 packed as
# 0xWWRRGGBB) and sets it live via the `ros2 param set` CLI.
#
# REQUIREMENTS / WHEN TO RUN:
#   The crazyflie_server must be running, i.e. start it first with:
#       ros2 launch crazyflie launch.py
#   and this shell must have ROS sourced (/opt/ros/$ROS_DISTRO/setup.bash and
#   install/setup.bash) with a matching ROS_DOMAIN_ID (unset/default on this rig).
#
# WHY THE CLI AND NOT rclpy:
#   On this rig, fresh rclpy processes (and crazyflie_py's Crazyswarm()) fail:
#   DDS discovery never completes so set_parameters/list_parameters service
#   calls time out, and Crazyswarm() hangs on all/emergency. The long-running
#   `ros2` CLI daemon keeps the ROS graph warm, so `ros2 param set` /
#   `ros2 param list` complete reliably. This script uses only that path.
#
# USAGE:
#   ./scripts/led.sh Yellow     # one-shot: set color and exit
#   ./scripts/led.sh Black      # LED off
#   ./scripts/led.sh            # interactive loop (prompt 'color> ', q to quit)

set -u

SERVER='/crazyflie_server'
LED_SUFFIX='params.colorLedBot.wrgb8888'
FALLBACK_PARAM='cf11.params.colorLedBot.wrgb8888'

# Named colors -> DECIMAL wrgb value (ros2 param set needs decimal integers).
# Values match src/crazyswarm2/crazyflie_examples/crazyflie_examples/color_led.py:
#   red=0xFF0000 yellow=0xFFFF00 green=0x00FF00 blue=0x0000FF
#   purple=0x9400D3 pink=0xFF69B4 white=0xFF000000 (W channel) black=0
color_value() {
    case "$1" in
        red)    echo 16711680 ;;   # 0xFF0000
        yellow) echo 16776960 ;;   # 0xFFFF00
        green)  echo 65280 ;;      # 0x00FF00
        blue)   echo 255 ;;        # 0x0000FF
        purple) echo 9699539 ;;    # 0x9400D3
        pink)   echo 16738740 ;;   # 0xFF69B4
        white)  echo 4278190080 ;; # 0xFF000000 dedicated W channel (RGB mix looks purplish)
        black)  echo 0 ;;          # 0x000000 (LED off)
        *)      return 1 ;;
    esac
}

VALID_COLORS='red yellow green blue purple pink white black'

# Discover every drone's LED param from the live server. Trims whitespace that
# `ros2 param list` indents its entries with.
discover_led_params() {
    ros2 param list "$SERVER" 2>/dev/null \
        | grep "$LED_SUFFIX" \
        | sed 's/^[[:space:]]*//; s/[[:space:]]*$//' \
        | grep -v '^$'
}

# Apply a color name to all discovered LED params. Returns non-zero on bad name.
apply_named() {
    local raw="$1"
    local name value
    name=$(printf '%s' "$raw" | tr '[:upper:]' '[:lower:]' | tr -d '[:space:]')

    if ! value=$(color_value "$name"); then
        echo "unknown color '$raw'. choose from: $VALID_COLORS"
        return 1
    fi

    # Discover drones fresh each time (a drone may (re)connect between calls).
    local params
    mapfile -t params < <(discover_led_params)

    if [ "${#params[@]}" -eq 0 ]; then
        echo "WARNING: no '$LED_SUFFIX' params found on $SERVER;" \
             "falling back to $FALLBACK_PARAM"
        params=("$FALLBACK_PARAM")
    fi

    local hex
    hex=$(printf '0x%08X' "$value")
    printf 'Setting %s (%s = %d) on %d drone(s):\n' \
        "$name" "$hex" "$value" "${#params[@]}"

    local p rc=0
    for p in "${params[@]}"; do
        if ros2 param set "$SERVER" "$p" "$value" >/dev/null 2>&1; then
            echo "  OK   $p"
        else
            echo "  FAIL $p"
            rc=1
        fi
    done
    return $rc
}

main() {
    if [ "$#" -ge 1 ]; then
        # One-shot mode.
        apply_named "$1"
        exit $?
    fi

    # Interactive mode.
    echo "Manual Color LED control. Colors: $VALID_COLORS"
    echo "Type a color name; 'q', 'quit', 'exit' or Ctrl-D to leave."
    while true; do
        if ! read -r -p 'color> ' line; then
            echo
            break
        fi
        local trimmed
        trimmed=$(printf '%s' "$line" | tr '[:upper:]' '[:lower:]' | tr -d '[:space:]')
        case "$trimmed" in
            q|quit|exit) break ;;
            '')          continue ;;
            *)           apply_named "$line" ;;
        esac
    done
}

main "$@"
