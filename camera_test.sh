#!/bin/bash - 
#===============================================================================
#
#          FILE: camera_test.sh
# 
#         USAGE: ./camera_test.sh 
# 
#   DESCRIPTION: 
# 
#       OPTIONS: ---
#  REQUIREMENTS: ---
#          BUGS: ---
#         NOTES: ---
#        AUTHOR: yanfeilong (), flyanb@isoftstone.com
#  ORGANIZATION: 
#       CREATED: 2013年09月24日 19:25
#      REVISION:  ---
#===============================================================================

set -o nounset                              # Treat unset variables as an error

#1. Primary camera snapshot test
./atomisp2_test -i 0 -c  -w 3264 -h 2448
./atomisp2_test -i 0 -c  -w 3264 -h 1836
./atomisp2_test -i 0 -c  -w 2560 -h 1920
./atomisp2_test -i 0 -c  -w 2048 -h 1536
./atomisp2_test -i 0 -c  -w 2048 -h 1152
./atomisp2_test -i 0 -c  -w 1920 -h 1080
./atomisp2_test -i 0 -c  -w 1600 -h 1200
./atomisp2_test -i 0 -c  -w 1280 -h 960
./atomisp2_test -i 0 -c  -w 1280 -h 720
./atomisp2_test -i 0 -c  -w 1024 -h 768
./atomisp2_test -i 0 -c  -w 640  -h 480

#2. Second camera snapshot test
./atomisp2_test -i 1 -c  -w 1280 -h 960
./atomisp2_test -i 1 -c  -w 1280 -h 720
./atomisp2_test -i 1 -c  -w 640 -h 480

#3. primary camera video record,not dump yuv file
./atomisp2_test -i 0 -r  -w 1920 -h 1080
./atomisp2_test -i 0 -r  -w 1280 -h 720
./atomisp2_test -i 0 -r  -w 720  -h 480
./atomisp2_test -i 0 -r  -w 640  -h 480
./atomisp2_test -i 0 -r  -w 320  -h 240
./atomisp2_test -i 0 -r  -w 176  -h 144

#4. second camera video record, not dump yuv file
./atomisp2_test -i 1 -r  -w 1280 -h 720
./atomisp2_test -i 1 -r  -w 720  -h 480
./atomisp2_test -i 1 -r  -w 640  -h 480
./atomisp2_test -i 1 -r  -w 320  -h 240
./atomisp2_test -i 1 -r  -w 176  -h 144

#5. primary camera video record,not 
./atomisp2_test -i 0 -r -d -w 1920 -h 1080
./atomisp2_test -i 0 -r -d -w 1280 -h 720
./atomisp2_test -i 0 -r -d -w 720  -h 480
./atomisp2_test -i 0 -r -d -w 640  -h 480
./atomisp2_test -i 0 -r -d -w 320  -h 240
./atomisp2_test -i 0 -r -d -w 176  -h 144

#6. second camera video record
./atomisp2_test -i 1 -r -d -w 1280 -h 720
./atomisp2_test -i 1 -r -d -w 720  -h 480
./atomisp2_test -i 1 -r -d -w 640  -h 480
./atomisp2_test -i 1 -r -d -w 320  -h 240
./atomisp2_test -i 1 -r -d -w 176  -h 144

#7. primary camera preview test
./atomisp2_test -i 0 -p
#8. second camera preview test
./atomisp2_test -i 1 -p

