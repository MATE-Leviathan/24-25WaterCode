#!/usr/bin/env bash
set -e

# If the 24-25WaterCode from outside the container is onwed
# by owner 1000, it is fine to use as is, since the id of
# ubuntu inside the container is also 1000
# If there's a mismatch, use bindsfs to remap the user and group
# to be ubuntu:ubuntu. Try to avoid this if possible since it
# seems to slow down builds in some cases
# Do the same for the tensorflow_workspace repo dir
id=$(stat -c '%u' /home/ubuntu/.24-25WaterCode.readonly/README.md)

if [[ $id == "1000" ]] ; then 
	if [[ ! -e /home/ubuntu/24-25WaterCode ]]; then
		ln -sf /home/ubuntu/.24-25WaterCode.readonly /home/ubuntu/24-25WaterCode
	fi
	if [[ ! -e /home/ubuntu/tensorflow_workspace ]]; then
		ln -sf /home/ubuntu/.tensorflow_workspace.readonly /home/ubuntu/tensorflow_workspace
	fi
else
	mkdir -p /home/ubuntu/24-25WaterCode

	# Might be best to fix sudoers file?
	# -p "" turns off prompt
	# -kS resets token and then reads password from stdin
	echo ubuntu | sudo -p "" -kS bindfs --force-user=ubuntu --force-group=ubuntu \
		--create-for-user=1000 --create-for-group=1000 \
		--chown-ignore --chgrp-ignore \
		/home/ubuntu/.24-25WaterCode.readonly /home/ubuntu/24-25WaterCode > /dev/null 2>&1

	mkdir -p /home/ubuntu/tensorflow_workspace

	# Might be best to fix sudoers file?
	# -p "" turns off prompt
	# -kS resets token and then reads password from stdin
	echo ubuntu | sudo -p "" -kS bindfs --force-user=ubuntu --force-group=ubuntu \
		--create-for-user=1000 --create-for-group=1000 \
		--chown-ignore --chgrp-ignore \
		/home/ubuntu/.tensorflow_workspace.readonly /home/ubuntu/tensorflow_workspace > /dev/null 2>&1
fi

exec "$@"
# Drop privileges and execute next container command, or 'bash' if not specified.
#if [[ $# -gt 0 ]]; then
   #exec sudo -u -H ubuntu -- "$@"
#else
   #exec sudo -u -H ubuntu -- bash
#fi