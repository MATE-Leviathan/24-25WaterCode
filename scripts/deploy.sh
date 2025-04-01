#!/bin/bash

#set -e
set -o pipefail

# This can be an array of IP address if there are multiple Jetsons
#JETSON_ADDR=( )
JETSON_ADDR=(10.50.2.100)

# Environment to deploy to (prod or dev).
INSTALL_ENV=dev

# Whether we're doing a build or just updating symlinks.
UPDATE_LINKS_ONLY=0

# Location of the code == location of deploy script
# Get directory where the deploy.sh script is located
# https://stackoverflow.com/questions/59895/getting-the-source-directory-of-a-bash-script-from-within
SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  THIS_DIR="$( cd -P "$( dirname "$SOURCE" )" >/dev/null && pwd )"
  SOURCE="$(readlink "$SOURCE")"
  [[ $SOURCE != /* ]] && SOURCE="$THIS_DIR/$SOURCE" # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
LOCAL_CLONE_LOCATION="$( cd -P "$( dirname "$SOURCE" )" >/dev/null && pwd )"
ROS_CODE_LOCATION=$LOCAL_CLONE_LOCATION/../fishROS_ws
RSYNC_OPTIONS="--delete"

# Command line argument parsing.
POSITIONAL=()
while [[ $# -gt 0 ]] ; do
    key="$1"

    case $key in
    -h|--help)
        usage
        exit 1
        ;;
    -p|--prod)
        INSTALL_ENV=prod
        shift
        ;;
    -d|--dev)
        INSTALL_ENV=dev
        shift
        ;;
    -o|--one-dir-sync)
        RSYNC_OPTIONS="--delete"
        shift
        ;;
    -t|--two-way-sync)
        RSYNC_OPTIONS=""
        shift
        ;;
    -u|--update-links-only)
        UPDATE_LINKS_ONLY=1
        shift
    ;;
    -b|--box)
        JETSON_ADDR=(10.50.2.100)
        shift
    ;;
    *) # unknown option
        POSITIONAL+=("$1") # save it in an array for later
        shift # past argument
        ;;
    esac
done
set -- "${POSITIONAL[@]}" # restore positional parameters


JETSON_CLONE_LOCATION=/home/ubuntu/24-25WaterCode
JETSON_ENV_LOCATION=$JETSON_CLONE_LOCATION.$INSTALL_ENV
JETSON_ROS_CODE_LOCATION=$JETSON_ENV_LOCATION/fishROS_ws

update_links() {

    for i in "${JETSON_ADDR[@]}"
    do
        ssh $i "rm $JETSON_CLONE_LOCATION && \
            ln -s $JETSON_ENV_LOCATION $JETSON_CLONE_LOCATION"
    done
    echo "Symlinks updated."
}

check_clockdiff() {
    #read -r -a TIMEDIFF <<< `clockdiff $1`
    #if [[ ${#TIMEDIFF[@]} -ge 3 ]]; then
    #    echo "${TIMEDIFF[1]} msec diff"
    #else
    #    echo "Could not parse clockdiff output!"
    #    exit 1
    #fi
    LOCALDATE=`date +%s`
    REMOTEDATE=`ssh $1 date +%s`
    let TIMEDIFF=$LOCALDATE-$REMOTEDATE
    TIMEDIFF=${TIMEDIFF#-}
    if [[ $TIMEDIFF -ge 600 ]]; then
        REMOTE_TIME=`ssh $1 date`
        echo -e "\e[1m\e[31mError\e[0m : Clock difference greater than 10 minutes."
        echo "    Local time: `date`"
        echo "    Time on $2: $REMOTE_TIME"
		echo "    Setting remote time to current host local time"
		echo ubuntu | ssh -tt $1 sudo date -s @$(date -u +"%s")
		echo ubuntu | ssh -tt $1 sudo hwclock -w
    fi
}

update_links
if [ $UPDATE_LINKS_ONLY -ne 0 ]; then
    exit 0
fi

echo "Checking time synchronization..."
for i in "${JETSON_ADDR[@]}"
do
    check_clockdiff "$i" "Jetson.$i"
done
echo "Time synchronized."

echo "Killing code on remotes "
for i in "${JETSON_ADDR[@]}"
do
    echo ubuntu | ssh -tt ubuntu@$i "sudo /home/ubuntu/24-25WaterCode/fishROS_ws/kill_ros_.sh"
done
echo "ROS Killed on Jetson"

# Copy from laptop to jetson.  Make it an actual sync - don't ignore
# newer files on the target as this causes problems when switching
# between git branches or between host laptops with different
# versions of code
for i in "${JETSON_ADDR[@]}"
do
    rsync -avzr $RSYNC_OPTIONS --no-times --checksum --exclude '.git' --exclude 'fishROS_ws/build*' \
        --exclude 'fishROS_ws/devel*' --exclude 'fishROS_ws/install*' --exclude 'fishROS_ws/logs*' \
        --exclude '*~' --exclude '*.sw[lmnop]' --exclude '*CMakeFiles*' \
        --exclude '*.avi' --exclude '*.exe'  --exclude 'pixy2' --exclude 'build' \
        --exclude '*.zms' --exclude '*.stl' --exclude '*.dae'  \
        --exclude 'fishROS_ws/.catkin_tools' \
        --exclude '.md5sum*txt' \
        --exclude '*.pt' --exclude '*.engine' \
	--exclude '*.deb' --exclude '*.whl' --exclude '*.tbz2' --exclude '*.dmg' --exclude '*.zip' \
	--exclude '*.nvvp' --exclude '*.qdrep' --exclude 'fishROS_ws/.catkin_tools'  --exclude 'TRT*bin'\
	--exclude '*.pyc'  --exclude '__pycache__' \
	--exclude 'bagfiles' --exclude '*.bag' --exclude '*.active' \
        $LOCAL_CLONE_LOCATION/../ $i:$JETSON_ENV_LOCATION/
    if [ $? -ne 0 ]; then
        echo -e "\e[1m\e[31mERROR\e[0m : Failed to synchronize source code TO $INSTALL_ENV on Jetson $i!"
        exit 1
    fi
done
echo "Synchronization to Jetson complete"


# Run Jetson native build(s) as a separate process(es).
JETSON_BUILD_PROCESSES=()
for i in "${JETSON_ADDR[@]}"
do
    echo "Starting Jetson $i native build using $JETSON_CLONE_LOCATION/fishROS_ws/native_build.sh" 
    (
        ssh -XC $i terminator -T \"Jetson $i\" -x "$JETSON_CLONE_LOCATION/fishROS_ws/native_build.sh || \
                                         read -p 'Jetson Build FAILED - press ENTER to close window'" && \
        echo "Jetson $i native build complete"
    ) &
    JETSON_BUILD_PROCESSES+=($!)
done

# Capture return code from Jetson build process(es)
# TODO - this doesn't actually capture the return code from
# the jetson native build, but instead from the terminator command
# which returns success if it was able to launch the command
JETSON_RCS=()
for i in "${JETSON_BUILD_PROCESSES[@]}"
do
    echo "Waiting for JETSON_BUILD_PROCESS $i"
    wait $i
    JETSON_RC=$?
    JETSON_RCS+=($JETSON_RC)
    echo " ... JETSON_BUILD_PROCESS $i returned $JETSON_RC"
done

# Print diagnostic info after all builds / deploys
# have run their course to make errors easier to see
EXIT_FAIL=0

# JETSON_RCS will be the return code for terminator
for i in "${JETSON_RCS[@]}"
do
    if [ $i -ne 0 ] ; then
        echo -e "Jetson build/deploy \e[1m\e[31mFAILED\e[0m"
        EXIT_FAIL=1
    fi
done


if [ $EXIT_FAIL -ne 0 ] ; then
    exit 1
fi

update_links
echo -e "\e[1m\e[32mFINISHED SUCCESSFULLY\e[0m"
