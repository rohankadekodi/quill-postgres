#!/bin/bash

if [ "$#" -ne 2 ]; then
    echo "Usage: ./dirtyTrackingOverhead.sh seq/rand runId"
    exit
fi

seqrand=$1
runid=$2
dirtyDataQuill=/home/rohan/projects/quill-modified
noDirtyDataQuill=/home/rohan/projects/quill-no-tracking
codeToRun=/home/rohan/projects/quill-microbenchmark/CachelineGranularity
flatTrackingQuill=/home/rohan/projects/quill-flat-tracking

# Compile Dirty Tracking Quill
cd $dirtyDataQuill
make clean
make

# Compile Non dirty Tracking Quill
cd $noDirtyDataQuill
make clean
make

# Compile Flat dirty Tracking Quill
cd $flatTrackingQuill
make clean
make

# Compile c code
cd $codeToRun
gcc motivational_fsync.c -o quillTesting

run_experiment()
{
    size=$1
    dir=$2
    dirtyTracking=$3
    huge=$4
    
    resultsDir=$dir/Results/Run$runid
    mkdir -p $resultsDir
    cd $dir
    
    for ctr in 1 2 3 4 5
    do
	rm /mnt/pmem_emul/writerand.txt && sync
	sleep 2
	taskset -c 3 ./run_quill.sh -p ./ -t nvp_nvp.tree $codeToRun/quillTesting $seqrand $size 1 $huge >> $resultsDir/$size$huge
	echo "$dirtyTracking $size Run $ctr done" 
    done
}


# main function
# With dirty data tracking

dir=$dirtyDataQuill
#for i in 4M 16M 64M 256M 400M
#do
i=512M
#   run_experiment $i $dir DirtyTracking Huge
run_experiment $i $dir DirtyTracking NoHuge
#done

# Without dirty data tracking

#dir=$noDirtyDataQuill
#for i in 4M 16M 64M 256M 400M
#do
#   run_experiment $i $dir NoDirtyTracking Huge
#    run_experiment $i $dir NoDirtyTracking NoHuge
#done

# Flat dirty data tracking

#dir=$flatTrackingQuill
#for i in 4M 16M 64M 256M 400M
#do
#    run_experiment $i $dir FlatDirtyTracking
#done