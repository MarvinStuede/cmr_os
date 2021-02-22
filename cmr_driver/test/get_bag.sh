#!/bin/bash
#Script to download and uncompress a bag file for testing
ppath=$(rospack find cmr_driver)
bagname=cmg_people_labor_3pers.bag
wget --no-verbose --content-disposition https://high-seas.projekt.uni-hannover.de/f/ca77e1a4aaa64711bbf2/?dl=1 -O "$ppath/test/$bagname"

