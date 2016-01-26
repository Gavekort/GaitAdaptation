# GaitAdaptation

Directories of interest: robdyn/src and sferes/exp/gatest

compile robdyn with ./waf configure and then ./waf

compile sferes with:
./waf configure --robdyn=/path/to/robdyn/bld/{debug,default}/src/ --robdyn-osg=/path/to/robdyn/bld/{debug,default}/src/ --includes=/path/to/robdyn/src/ --cpp11=yes
