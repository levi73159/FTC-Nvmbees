#!/usr/bin/zsh

./gradlew assembleDebug
adb $1 install -r TeamCode/build/outputs/apk/debug/TeamCode-debug.apk