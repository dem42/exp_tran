#!/bin/sh

INPUT_DIRECTORY=$1
OUT_FILE=$2

echo "building using $INPUT_DIRECTORY as database"
rm -rf /tmp/list.txt
ls $INPUT_DIRECTORY > /tmp/list.txt
egrep "(AN04|DI04|FE04|NE00|HA04|SA04|SU04)" /tmp/list.txt >> /tmp/greped_list.txt 

touch $OUT_FILE
echo `grep -c . /tmp/greped_list.txt` > $OUT_FILE
cat /tmp/greped_list.txt >> $OUT_FILE

rm -rf /tmp/greped_list.txt
rm -rf /tmp/list.txt

 
