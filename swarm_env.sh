#!/usr/bin/env bash
# =============================================================================
# swarm_env.sh -- set up ONE terminal for this project, correctly, on either
# machine. SOURCE it, do not execute it:
#
#     source ~/swarm_project/swarm_env.sh              # on the robot
#     source ~/swarm/swarm_project/swarm_env.sh        # on the workstation
#
# It does, in the only order that works:
#   1. source ROS2 for whichever distro this machine has
#   2. source the workspace's install/ (so `ros2 pkg prefix` can resolve)
#   3. export the DDS variables, picking the config file for THIS distro
#   4. check the config actually exists, and say so plainly
#
# WHY THIS EXISTS. The DDS exports have to come AFTER the workspace is sourced,
# because CYCLONEDDS_URI is built from `ros2 pkg prefix swarm_network`. Get the
# order wrong and the substitution is empty, leaving file:///share/... -- then
# every ROS2 process in that terminal dies in rmw_create_node.
#
# And get them MISSING and it is quieter still: the process joins the default
# domain with the default rmw, runs perfectly, logs nothing wrong, and is simply
# invisible to the other machine. That is what happened to block_detector_node
# on 2026-08-20 -- it sat there logging "serving /detect_block" while mars timed
# out waiting for exactly that service.
#
# Every terminal needs this, including the ones that run a bare `python3
# script.py` rather than `ros2 launch`. Those are the easy ones to forget.
# =============================================================================

# --- must be sourced, not executed -------------------------------------------
if [ "${BASH_SOURCE[0]}" = "${0}" ]; then
    echo "ERROR: source this script, do not run it:"
    echo "    source ${0}"
    echo "(Running it sets variables in a child shell that exits immediately.)"
    exit 1
fi

_swarm_env() {   # a function so locals do not leak into the user's shell
    local ws distro ros_setup cfg

    ws="$(cd "$(dirname "${BASH_SOURCE[1]}")" && pwd)"

    # 1. ROS2. Prefer an already-sourced distro; otherwise take what exists.
    if [ -n "${ROS_DISTRO:-}" ] && [ -d "/opt/ros/${ROS_DISTRO}" ]; then
        distro="$ROS_DISTRO"
    else
        for d in galactic jazzy humble; do
            [ -d "/opt/ros/$d" ] && { distro="$d"; break; }
        done
    fi
    if [ -z "${distro:-}" ]; then
        echo "[swarm_env] ERROR: no ROS2 found under /opt/ros. Run the installer first."
        return 1
    fi

    ros_setup="/opt/ros/${distro}/setup.bash"

    # nounset has to come OFF across ROS's setup.bash: its line 8 reads
    # $AMENT_TRACE_SETUP_FILES with no default, and `set -u` makes that fatal.
    #
    # RESTORE IT, do not just switch it on afterwards. This script is SOURCED,
    # so `set -u` here lands in the user's interactive shell -- where nounset is
    # normally OFF and must stay off. bash-completion uses ${!ref} indirect
    # expansion internally, which under nounset makes every Tab press print
    #     bash: !ref: unbound variable
    # and complete nothing. That is a bug this script caused on 2026-08-20.
    _swarm_had_u=0
    case "$-" in *u*) _swarm_had_u=1 ;; esac
    set +u

    # shellcheck disable=SC1090
    . "$ros_setup"

    # 2. The workspace, so `ros2 pkg prefix` can resolve swarm_network.
    if [ -f "$ws/install/setup.bash" ]; then
        # shellcheck disable=SC1090
        . "$ws/install/setup.bash"
    else
        echo "[swarm_env] WARNING: $ws/install/setup.bash not found -- build first:"
        echo "               cd $ws && colcon build"
        [ "$_swarm_had_u" = 1 ] && set -u
        unset _swarm_had_u
        return 1
    fi
    [ "$_swarm_had_u" = 1 ] && set -u
    unset _swarm_had_u

    # 3. DDS. The config filename is per-distro: Galactic and Jazzy ship
    # different Cyclone versions that need different settings, and the stale
    # suffix-less cyclonedds.xml is valid for neither.
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export ROS_DOMAIN_ID="${SWARM_DOMAIN_ID:-42}"

    local prefix
    prefix="$(ros2 pkg prefix swarm_network 2>/dev/null)"
    if [ -z "$prefix" ]; then
        echo "[swarm_env] ERROR: swarm_network is not on the search path even after"
        echo "            sourcing install/. Build it:"
        echo "               cd $ws && colcon build --packages-select swarm_network"
        return 1
    fi

    cfg="${prefix}/share/swarm_network/config/cyclonedds_${distro}.xml"
    if [ ! -f "$cfg" ]; then
        echo "[swarm_env] ERROR: no DDS config for this distro at:"
        echo "            $cfg"
        echo "            swarm_network has configs for: $(ls "${prefix}/share/swarm_network/config/" 2>/dev/null | tr '\n' ' ')"
        return 1
    fi
    export CYCLONEDDS_URI="file://${cfg}"

    # 4. Report, including the peers, since a wrong IP is the other silent failure.
    echo "[swarm_env] ROS2 ${distro}  |  workspace $ws"
    echo "[swarm_env] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}  RMW=${RMW_IMPLEMENTATION}"
    echo "[swarm_env] CYCLONEDDS_URI -> ${cfg}"
    local peers
    peers="$(grep -oE '<Peer address="[^"]+"' "$cfg" 2>/dev/null | sed 's/.*"\(.*\)"/\1/' | tr '\n' ' ')"
    echo "[swarm_env] peers: ${peers:-none found}"
    echo "[swarm_env] this machine: $(hostname -I 2>/dev/null | tr -s ' ')"
    echo "[swarm_env] ready. If this machine's IP is not in the peer list above,"
    echo "            edit src/swarm_network/config/*.xml and rebuild swarm_network."
    return 0
}

_swarm_env
unset -f _swarm_env
