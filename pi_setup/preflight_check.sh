#!/usr/bin/env bash
# =============================================================================
# preflight_check.sh -- verify this machine can actually `colcon build` this
# workspace, BEFORE colcon buries the reason in a CMake stack trace.
#
# Written after GitHub issue #22, where a fresh arm produced two successive
# walls of CMake output ("Findament_cmake.cmake" missing, then
# "Findhardware_interface.cmake" missing) for two mundane causes: ROS was
# never sourced in that shell, and the ros2_control apt packages were never
# installed because install_pi_galactic.sh had not been run. Both are one-line
# fixes; neither is discoverable from what CMake prints.
#
# Safe to run anywhere, any time. Read-only -- installs nothing, builds
# nothing, and touches no hardware. Run it from the workspace root:
#
#   ./pi_setup/preflight_check.sh
#
# Exit status: 0 if the workspace should build, 1 if something is missing.
# =============================================================================

set -uo pipefail   # deliberately NOT -e: we want to report every problem in
                   # one pass, not stop at the first one.

FAIL=0
note()  { echo "  [OK]   $*"; }
bad()   { echo "  [FAIL] $*"; FAIL=1; }
warn()  { echo "  [WARN] $*"; }
step()  { echo; echo "── $* ──────────────────────────────────"; }

echo "=============================================="
echo " swarm_project build preflight check"
echo "=============================================="

# ─── 1. IS ROS SOURCED AT ALL? ────────────────────────────────────────────────
# This gates everything below -- `ros2 pkg prefix` does not exist until ROS is
# on PATH, so an unsourced shell would otherwise report every package missing.
step "1. ROS2 environment"

if [[ -z "${ROS_DISTRO:-}" ]] || ! command -v ros2 >/dev/null 2>&1; then
    bad "No ROS2 environment in this shell (ROS_DISTRO unset / no ros2 on PATH)."
    echo
    echo "  This is the cause of:  CMake Error ... \"Findament_cmake.cmake\""
    echo "                         ros2: command not found"
    echo
    echo "  Fix -- source ROS, then re-run this check:"
    for d in /opt/ros/*/setup.bash; do
        [[ -e "$d" ]] && echo "      source $d"
    done
    echo
    echo "  Sourcing is PER-TERMINAL and does not persist. Every new shell you"
    echo "  build or run in needs it again."
    echo
    echo "  Do NOT source /opt/ros/noetic (ROS1, vendor image) in the same"
    echo "  shell -- Noetic and Galactic conflict."
    echo
    echo "Preflight FAILED -- stopping here; later checks need ROS on PATH."
    exit 1
fi
note "ROS2 $ROS_DISTRO sourced."

if [[ -n "${ROS_VERSION:-}" && "${ROS_VERSION}" != "2" ]]; then
    bad "ROS_VERSION=$ROS_VERSION -- a ROS1 install is sourced in this shell."
fi

command -v colcon >/dev/null 2>&1 \
    && note "colcon present." \
    || bad "colcon missing. Fix: sudo apt install -y python3-colcon-common-extensions"

# ─── 2. REQUIRED ROS PACKAGES ─────────────────────────────────────────────────
# Checked by ament index lookup rather than dpkg, so a source-built package
# counts as present just like an apt one.
step "2. ROS packages this workspace builds/links against"

have_pkg() { ros2 pkg prefix "$1" >/dev/null 2>&1; }

check_pkgs() {
    local label="$1"; shift
    local missing=()
    for p in "$@"; do have_pkg "$p" || missing+=("$p"); done
    if [[ ${#missing[@]} -eq 0 ]]; then
        note "$label: all present."
    else
        bad "$label: missing ${missing[*]}"
        # Map ROS package names to their apt package names: apt uses hyphens,
        # and the metapackages are named differently from what CMake asks for.
        local apt=()
        for m in "${missing[@]}"; do
            case "$m" in
                hardware_interface|controller_manager) apt+=("ros-${ROS_DISTRO}-ros2-control") ;;
                joint_state_broadcaster|joint_trajectory_controller) apt+=("ros-${ROS_DISTRO}-ros2-controllers") ;;
                *) apt+=("ros-${ROS_DISTRO}-${m//_/-}") ;;
            esac
        done
        # shellcheck disable=SC2207
        local uniq=($(printf '%s\n' "${apt[@]}" | sort -u))
        echo "         Fix: sudo apt install -y ${uniq[*]}"
    fi
}

check_pkgs "build tooling"  ament_cmake rosidl_default_generators
check_pkgs "MoveIt2"        moveit_ros_move_group moveit_kinematics moveit_configs_utils
check_pkgs "robot state"    xacro robot_state_publisher tf2_ros
check_pkgs "DDS"            rmw_cyclonedds_cpp

# ros2_control is Galactic-only in this project: mycobot_hardware is written
# against Galactic's hardware_interface read()/write() signature and is skipped
# on Jazzy, so its dependencies are not required there.
if [[ "$ROS_DISTRO" == "galactic" ]]; then
    check_pkgs "ros2_control (needed by mycobot_hardware)" \
        hardware_interface controller_manager pluginlib \
        joint_state_broadcaster joint_trajectory_controller
    echo "         ^ this is the cause of: \"Findhardware_interface.cmake\" missing"
else
    warn "Not Galactic ($ROS_DISTRO) -- skipping ros2_control checks."
    warn "Build here with: colcon build --packages-skip mycobot_hardware"
fi

# ─── 2b. DDS ENVIRONMENT ──────────────────────────────────────────────────────
# A CYCLONEDDS_URI pointing at a file that is not there is worse than one that
# is unset: Cyclone refuses to create a domain and EVERY ROS2 process in the
# shell dies in rmw_create_node, controller spawners included. The usual cause
# is exporting it before `source install/setup.bash`, so the
# `$(ros2 pkg prefix swarm_network)` substitution came back empty and left
# file:///share/... behind.
step "2b. DDS environment"

if [[ -z "${CYCLONEDDS_URI:-}" ]]; then
    note "CYCLONEDDS_URI unset (fine for a local-only session; set it before cross-machine work)."
else
    dds_path="${CYCLONEDDS_URI#file://}"
    if [[ -f "$dds_path" ]]; then
        note "CYCLONEDDS_URI -> $dds_path"
        case "$dds_path" in
            *cyclonedds.xml)
                bad "that is the STALE pre-split config, not valid for either distro."
                echo "         Use cyclonedds_${ROS_DISTRO}.xml instead." ;;
            *cyclonedds_${ROS_DISTRO}.xml)
                note "and it is the right file for $ROS_DISTRO." ;;
            *)
                warn "expected cyclonedds_${ROS_DISTRO}.xml for this distro." ;;
        esac
    else
        bad "CYCLONEDDS_URI points at a file that does not exist:"
        echo "         $CYCLONEDDS_URI"
        if [[ "$dds_path" == /share/* ]]; then
            echo "         The path starts at /share, so \$(ros2 pkg prefix swarm_network)"
            echo "         expanded to NOTHING. You exported this before sourcing the"
            echo "         workspace. Every ROS2 node in this shell will die in"
            echo "         rmw_create_node until it is fixed. In this order:"
            echo "             colcon build --packages-select swarm_network"
            echo "             source install/setup.bash"
            echo "             export CYCLONEDDS_URI=file://\$(ros2 pkg prefix swarm_network)/share/swarm_network/config/cyclonedds_${ROS_DISTRO}.xml"
            echo "         If that export lives in ~/.bashrc, it runs before the"
            echo "         workspace is sourced there too -- move it after, or"
            echo "         hardcode the full path."
        fi
    fi
fi

# Cross-machine work needs both ends on the same domain.
if [[ -n "${RMW_IMPLEMENTATION:-}" && "$RMW_IMPLEMENTATION" != "rmw_cyclonedds_cpp" ]]; then
    warn "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION (expected rmw_cyclonedds_cpp)."
fi


# ─── 3. PYMYCOBOT ─────────────────────────────────────────────────────────────
# Not a rosdep/apt dependency -- pip only, via pi_setup/requirements.txt --
# so nothing in the build catches its absence. It fails at runtime instead.
step "3. pymycobot (robot serial driver, pip-installed)"

if [[ "$ROS_DISTRO" == "galactic" ]]; then
    python3 - <<'PY' || true
try:
    import pymycobot
    v = getattr(pymycobot, "__version__", "unknown")
    try:
        from packaging import version
        ok = version.parse(v) >= version.parse("3.6.1")
    except Exception:
        ok = None
    if ok is False:
        print(f"  [WARN] pymycobot {v} < 3.6.1. Fix: pip3 install -U pymycobot")
    else:
        print(f"  [OK]   pymycobot {v}.")
except ImportError:
    print("  [WARN] pymycobot not installed -- the arm will not move.")
    print("         Fix: python3 -m pip install --user -r pi_setup/requirements.txt")
PY
else
    warn "Not Galactic -- pymycobot not needed on this machine."
fi

# ─── 4. WORKSPACE SANITY ──────────────────────────────────────────────────────
step "4. Workspace layout"

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# The legacy/ tree holds an old AGV package named "control". Without a
# COLCON_IGNORE marker colcon builds it as a workspace package, which is what
# made issue #22's build log mention a package that isn't in src/.
if [[ -e "$WS_ROOT/legacy/COLCON_IGNORE" ]]; then
    note "legacy/ is colcon-ignored."
else
    bad "legacy/COLCON_IGNORE missing -- colcon will build the dead 'control' package."
    echo "         Fix: touch $WS_ROOT/legacy/COLCON_IGNORE"
fi

# Adding COLCON_IGNORE stops future builds of "control" but does not remove
# what earlier builds already produced. Stale install/control stays on
# AMENT_PREFIX_PATH once install/setup.bash is sourced, so `ros2 pkg list` keeps
# showing a package that no longer exists in the workspace. Harmless, but it
# muddies exactly the kind of debugging that produced issue #22.
stale=()
for d in build/control install/control; do
    [[ -e "$WS_ROOT/$d" ]] && stale+=("$d")
done
if [[ ${#stale[@]} -gt 0 ]]; then
    warn "stale artifacts from the old 'control' package: ${stale[*]}"
    echo "         These predate legacy/COLCON_IGNORE and will not rebuild."
    echo "         Optional cleanup: rm -rf ${stale[*]/#/$WS_ROOT/}"
fi

EXPECTED=(mycobot_description mycobot_280pi_camera_moveit2 mycobot_hardware
          swarm_interfaces swarm_network swarm_pkg)
missing_src=()
for p in "${EXPECTED[@]}"; do
    [[ -d "$WS_ROOT/src/$p" ]] || missing_src+=("$p")
done
if [[ ${#missing_src[@]} -eq 0 ]]; then
    note "all ${#EXPECTED[@]} source packages present."
else
    bad "src/ is missing: ${missing_src[*]}"
    echo "         Your checkout is behind. Fix: git pull"
fi

# ─── SUMMARY ──────────────────────────────────────────────────────────────────
echo
echo "=============================================="
if [[ $FAIL -eq 0 ]]; then
    echo " PREFLIGHT PASSED -- safe to colcon build"
    echo "=============================================="
    if [[ "$ROS_DISTRO" == "galactic" ]]; then
        echo "    cd $WS_ROOT && colcon build && source install/setup.bash"
    else
        echo "    cd $WS_ROOT && colcon build --packages-skip mycobot_hardware"
        echo "    source install/setup.bash"
    fi
else
    echo " PREFLIGHT FAILED -- fix the [FAIL] lines above first"
    echo "=============================================="
    echo
    echo " On a fresh arm the usual answer is to run the installer, which"
    echo " handles all of the apt packages above in one pass:"
    echo "     ./pi_setup/install_pi_galactic.sh"
fi
echo
exit $FAIL
