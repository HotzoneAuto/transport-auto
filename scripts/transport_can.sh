DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cd "${DIR}/.."

source "${DIR}/apollo_base.sh"

   function start() {
    LOG="${APOLLO_ROOT_DIR}/data/log/transport_can.out"
    CMD="cyber_launch start /apollo/modules/canbus/launch/transport_can.launch"
    NUM_PROCESSES="$(pgrep -c -f "modules/canbus/dag/transport_can.dag")"
    if [ "${NUM_PROCESSES}" -eq 0 ]; then
        eval "nohup ${CMD} </dev/null >${LOG} 2>&1 &"
    fi
}

function stop() {
    eval "nohup cyber_launch stop /apollo/modules/canbus/launch/transport_can.launch < /dev/null 2>&1 &"
}

# run command_name module_name
function run() {
    case $1 in
        start)
            start
            ;;
        stop)
            stop
            ;;
        *)
            start
            ;;
    esac
}

run "$1"
