#!/bin/bash

sscanf() {
    local str="$1"
    local format="$2"
    [[ "$str" =~ $format ]]
}

# Files location.
main_file="../application/src/main.c"
version_file="../application/inc/version.h"

# Touch files to force makefile operation.
touch $main_file
touch $version_file

# Get current date.
date=`date -R`

# Execute git commands.
if [[ -n "$GIT_DESCRIBE_OVERRIDE" ]]; then
    echo "[git_version.sh] Using GIT_DESCRIBE_OVERRIDE: $GIT_DESCRIBE_OVERRIDE"
    git_version="$GIT_DESCRIBE_OVERRIDE"
else
    echo "[git_version.sh] Using git describe"
    git_version=`git describe --long --always`
fi
diff=`git diff ':(exclude)../application/inc/spsws_flags.h' ':(exclude)../script'`

# Extract fields
sscanf $git_version "sw([0-9]+).([0-9]+)-([0-9]+)-g(.+)"

# Manage dirty flag.
dirty_flag=0
if [[ $diff ]]; then
    dirty_flag=1
fi

echo "/*" > $version_file
echo " * version.h" >> $version_file
echo " *" >> $version_file
echo " * Auto-generated on: $date" >> $version_file
echo " * Author: Ludo" >> $version_file
echo " */" >> $version_file
echo "" >> $version_file
echo "#ifndef __VERSION_H__" >> $version_file
echo "#define __VERSION_H__" >> $version_file
echo "" >> $version_file
echo "#define GIT_VERSION       \"$git_version\"" >> $version_file
echo "#define GIT_MAJOR_VERSION ${BASH_REMATCH[1]}" >> $version_file
echo "#define GIT_MINOR_VERSION ${BASH_REMATCH[2]}" >> $version_file
echo "#define GIT_COMMIT_INDEX  ${BASH_REMATCH[3]}" >> $version_file
echo "#define GIT_COMMIT_ID     0x${BASH_REMATCH[4]}" >> $version_file
echo "#define GIT_DIRTY_FLAG    $dirty_flag" >> $version_file
echo "" >> $version_file
echo "#endif /* __VERSION_H__ */" >> $version_file
