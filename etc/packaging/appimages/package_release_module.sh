#!/bin/bash

set -e
SELF=$(realpath $0)
source "$(dirname $SELF)/utils.sh"

if get_stable_version_tag > /dev/null
then
	BUILD_CHANNEL="stable" appimage-builder --recipe cartographer-module-`uname -m`.yml
	BUILD_CHANNEL=$(get_stable_version_tag) appimage-builder --recipe cartographer-module-`uname -m`.yml
elif get_rc_version_tag > /dev/null
then
	BUILD_CHANNEL=$(get_rc_version_tag) appimage-builder --recipe cartographer-module-`uname -m`.yml
else
	BUILD_CHANNEL="latest" appimage-builder --recipe cartographer-module-`uname -m`.yml
fi
