#!/bin/bash

if [ "$#" -ne 2 ]; then
    echo "Usage: ./dirtyTrackingOverhead.sh <Run ID> <rwritefsync>"
    exit
fi

runid=$1
workload=$2
dirtyDataQuill=/home/rohan/projects/quill-modified
noDirtyDataQuill=/home/rohan/projects/quill-no-tracking
flatTrackingQuill=/home/rohan/projects/quill-flat-tracking
filebench=/home/rohan/projects/filebench
workloadFile=webserver.f
ext4DAXDir=/home/rohan/projects/ext4DAX

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

run_experiment()
{
    dir=$1
    dirtyTracking=$2
    workloadResDir=$3
    
    resultsDir=$dir/Results/Filebench/$workloadResDir/
    mkdir -p $resultsDir
    cd $dir
    
    for ctr in 1 2 3 4 5
    do
	rm -rf /mnt/pmem_emul/* && sync
	sleep 5
	if [ "$dirtyTracking" = "DirtyTracking" ]; then
		./run_quill.sh -p ./ -t nvp_nvp.tree $filebench/filebench -f $filebench/workloads/$workloadFile >> $resultsDir/Run$runid
	else
		$filebench/filebench -f $filebench/workloads/$workloadFile >> $resultsDir/Run$runid
	fi	

	echo "$dirtyTracking filebench Run $ctr done" 
    done
}

setup_expt()
{
    dir=$1
    dirtyTracking=$2
    
    workloadFile=webserver.f
    echo "Running $workloadFile"
    run_experiment $dir $dirtyTracking webserver
    
}
    
# main function
# With dirty data tracking
dir=$dirtyDataQuill
setup_expt $dir DirtyTracking     

# Without dirty data tracking

dir=$ext4DAXDir
setup_expt $dir NoDirtyTracking 

# Flat dirty data tracking

#dir=$flatTrackingQuill
#for i in 4M 16M 64M 256M 400M
#do
#    run_experiment $i $dir FlatDirtyTracking
#done
