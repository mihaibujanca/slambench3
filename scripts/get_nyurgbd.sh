#!/bin/bash

set -x

dataset_dir="datasets/NYURGBDv2"

base_dataset_url="http://horatio.cs.nyu.edu/mit/silberman/nyu_depth_v2"

labelled="nyu_depth_v2_labeled.mat"
labelled_dataset_url="http://horatio.cs.nyu.edu/mit/silberman/nyu_depth_v2/${labelled}"

toolbox="toolbox_nyu_depth_v2.zip"
toolbox_url="http://cs.nyu.edu/~silberman/code/${toolbox}"
toolbox_dir="nyu_toolbox"

sequence=$( echo $1 | cut -d/ -f3 | cut -d. -f1 )
scene=$( echo $sequence | cut -d_ -f1 )

raw_dir="${scene}_raw"
processed_dir="${scene}_processed"

if [ ! -d "${dataset_dir}/${processed_dir}/${sequence}" ]; then
    mkdir -p $dataset_dir
    pushd $dataset_dir

    if [ ! -d ${toolbox_dir} ]; then
	mkdir -p ${toolbox_dir}
	wget -t2 -d ${toolbox_url}
	unzip ${toolbox} -d ${toolbox_dir}
	cp "../../nyu_toolbox/nyu_process.m" ${toolbox_dir}
	cp "../../nyu_toolbox/fill_depth_colorization.m" ${toolbox_dir}
	rm ${toolbox}
    fi

    if [ ! -f "${labelled}" ]; then
	wget -t2 $labelled_dataset_url
    fi

    part_no=1

    mkdir -p ${raw_dir}
    pushd ${raw_dir}

    while : ; do
	dir="${scene}s_part${part_no}"
	file="${scene}s_part${part_no}.zip"
	url="${base_dataset_url}/${file}"
	(( part_no+=1 ))

	if [ ! -d ${dir} ] && [ ! -f ${file} ]; then
	    wget -t2 $url
	    returncode=$?
	    [[ $returncode -eq 0 ]] || break
	    mkdir ${dir}
	    unzip ${file} -d ${dir}
	    rm -f ${file}
	fi
    done

    popd

    finalScenePath=$( find . -name ${sequence} -exec dirname {} \; | cut -c 3- )

    processedDir="${scene}_processed"
    mkdir -p ${processedDir}/${sequence}

    popd
    pushd "${dataset_dir}/nyu_toolbox"

    matlab -nodisplay -nodesktop -r "compile;nyu_process('../${finalScenePath}','${sequence}','../${labelled}','../${processedDir}');exit;"

    popd

else
    echo "Sequence ${sequence} is already processed; skipping to dataset generation"
fi

./build/bin/dataset-generator -d nyurgbd -i  ${dataset_dir}/${processed_dir}/${sequence} -o $1

echo "DONE"



