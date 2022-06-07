#!/usr/bin/env bash

# Synchronizes repo from a development machine to a minipc.
#
# Reads ./sync_development.config for configuration. Edit
# ./sync_development.config for your environment.
#
#
# macOS setup instructions:
# > brew install fswatch
# > brew install rsync
#
# throw errors on undefined variables. Useful when configuration is missing
# variables.
set -u

DIR="${0%/*}"

load_config() {
    # throw an error if config does not exist
    set -e
    . "$DIR/sync_development.config"
    set +e
}

run_rsync() {
    rsync \
      --timeout=5 \
      --progress \
      --info=progress2 \
      --info=del1 \
      --archive \
      --update \
      --delete \
      --exclude ".git/*" \
      --exclude "build/*" \
      --exclude "ORB_SLAM3/lib/libORB_SLAM3.so" \
      --exclude "ORB_SLAM3/Thirdparty/*" \
      --exclude "ORB_SLAM3/Examples_old/*" \
      --exclude "ORB_SLAM3/Vocabulary/*" \
      -e "ssh -p $TARGET_PORT" \
      "$SOURCE" \
      "$TARGET"

    if [[ $? -eq 0 ]]; then
        echo "Sync complete."
    else
        echo "Sync failed."
    fi
}

load_config
run_rsync

fswatch -o "$SOURCE" | while read f; do run_rsync; done
