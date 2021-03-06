#!/bin/sh

#********************************************
# File structure
#********************************************
# 0. Set variables
# 1. Locates $SNS
# 2. Create channels
# 3. Define some useful functions
# 4. Set script options
#********************************************

#************************
# 0. Set variables
#************************

# ARMS + TORSO: SocketCAN interfaces
CAN_L=can0
CAN_R=can1


#***********************
# 1. LOCATE $SNS
#***********************

# If $SNS string is empty:
if test -z $SNS; then
    # If sns in this path has executable permissions
    if test -x ~/local/etc/init.d/sns; then
	SNS=~/local/etc/init.d/sns
    elif test -x /usr/local/etc/init.d/sns; then
	SNS=/usr/local/etc/init.d/sns
    elif test -x /etc/init.d/sns; then
	SNS=/etc/init.d/sns
    else
	echo "[ERROR] Could not find SNS program in any of the default locations"
	exit 1
    fi
fi

#*************************
# 2. CREATE CHANNELS
#*************************
CHANNELS="ref-left state-left ref-right state-right"

#******************************
# 3. DEFINE USEFUL FUNCTIONS
#******************************

# Create channels for arm control
oldschunk_ach_mk() {
    for c in $CHANNELS; do
	ach mk -1 -o 666 $c
    done 
}

# Remove channels for arm control
oldschunk_ach_rm() {
    for c in $CHANNELS; do
	ach rm $c
    done
}

# Start: Create channels + run daemons
oldschunk_start() {
    
    # Create channels
    oldschunk_ach_mk

    # Run daemons can402 for left and right arm
    $SNS run -d -r lwa-left -- \
	pcio-sns -f $CAN_L -R 1 -C 0 -n 3 -n 4 -n 5 -n 6 -n 7 -n 8 -n 9 -c ref-left -s state-left -v
    $SNS run -d -r lwa-right -- \
	pcio-sns -f $CAN_R -R 1 -C 0 -n 3 -n 4 -n 5 -n 6 -n 7 -n 8 -n 9 -c ref-right -s state-right -v
}

# Expunge: Remove temporal folders for log
oldschunk_expunge() {
    sudo rm -rf /var/tmp/sns/lwa-left
    sudo rm -rf /var/tmp/sns/lwa-right

    sudo rm -rf /var/run/sns/lwa-left
    sudo rm -rf /var/run/sns/lwa-right
}

# Stop: Stop daemons and programs
oldschunk_stop() {
    $SNS kill lwa-left
    $SNS kill lwa-right
}

# Change ownerships of SNS temporal files
oldschunk_steal() {
    chown -R $1 /var/run/sns/lwa-left \
	/var/run/sns/lwa-right \
	/var/tmp/sns/lwa-left \
	/var/tmp/sns/lwa-right  
}

#*************************
# 4. Set script options
#**************************

case "$1" in
    start)
	oldschunk_start
	;;
    stop)
	oldschunk_stop
	;;
    rm)
	oldschunk_ach_rm
	;;
    mk) cricthon_arms_ach_mk
	;;
    steal)
	shift
	oldschunk_steal $@
	;;
    expunge)
	oldschunk_expunge
	;;
    *)
	echo "[ERROR] Invalid command. Options are start / stop / rm / mk / steal NEW_OWNER / expunge"
	exit 1
	;;
esac

