#!/bin/bash

# Function to parse version and extract components into named variables
parse_git_version() {
    local str="$1"
    # Try to match the field version format first: sw.X.Y.Z.f-<commits>-g<hash>
    if [[ "$str" =~ ^sw([0-9]+)\.([0-9]+)\.([0-9]+)\.f-([0-9]+)-g([a-f0-9]+) ]]; then
        version_major="${BASH_REMATCH[1]}"
        version_minor="${BASH_REMATCH[2]}"
        version_patch="${BASH_REMATCH[3]}"
        commit_index="${BASH_REMATCH[4]}"
        commit_id="${BASH_REMATCH[5]}"
        return 0
    fi
    # Fallback to standard version format: sw.X.Y-<commits>-g<hash>
    if [[ "$str" =~ ^sw([0-9]+)\.([0-9]+)-([0-9]+)-g([a-f0-9]+) ]]; then
        version_major="${BASH_REMATCH[1]}"
        version_minor="${BASH_REMATCH[2]}"
        version_patch="0"
        commit_index="${BASH_REMATCH[3]}"
        commit_id="${BASH_REMATCH[4]}"
        return 0
    fi
    return 1
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
parse_git_version "$git_version"
if [[ $? -ne 0 ]]; then
    echo "[git_version.sh] Warning: could not parse version $git_version"
    exit 1
fi

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
echo "#define GIT_MAJOR_VERSION $version_major" >> $version_file
echo "#define GIT_MINOR_VERSION $version_minor" >> $version_file
echo "#define GIT_COMMIT_INDEX  $commit_index" >> $version_file
echo "#define GIT_COMMIT_ID     0x$commit_id" >> $version_file
echo "#define GIT_DIRTY_FLAG    $dirty_flag" >> $version_file
echo "" >> $version_file
echo "#endif /* __VERSION_H__ */" >> $version_file
