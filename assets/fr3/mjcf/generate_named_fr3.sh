#!/bin/sh
script_dir=$(dirname $(realpath $0))
id=${1}
dst_dir=${2}

sed -E -e "s/name=\"([^\"]*)/name=\"\1_${id}/" \
       -e "s/joint=\"([^\"]*)/joint=\"\1_${id}/" \
       -e "s/joint1=\"([^\"]*)/joint1=\"\1_${id}/" \
       -e "s/joint2=\"([^\"]*)/joint2=\"\1_${id}/" \
       -e "s/tendon=\"([^\"]*)/tendon=\"\1_${id}/" \
       "${script_dir}/fr3_unnamed.xml" > "$(realpath ${dst_dir})/fr3_${id}.xml"
