# GaitAdaptation

This is the code belonging to my master thesis "Evolving Gait Patterns using Evolutionary Algorithms and Intelligent Trial and Error".

I would like to thank the Resibots team, the authors of Sferes2, Robdyn and Limbo. The frameworks used can be found here:
* https://github.com/sferes2/sferes2
* https://github.com/resibots/limbo
* https://github.com/resibots/robdyn
* https://github.com/sferes2/map_elites
* https://github.com/resibots/ITE/

## Directories of interest
Those interested in my code should look at the following directories:
* ./limbo_old/exp/gaitopt/ 
* ./robdyn/src/{robot,demos}/
* ./sferes2/exp/gatest/


**GaitAdapt (aka gatest)**: Evolves a MapElites archive for the robot4, produces archive_$gen.dat that can be used by Limbo

**GaitOpt**: IT&E-algorithm that searches the MapElites archive for the most fitting elites for a given situation

**Robot4**: Defines the robot used, check out ./robdyn/src/robot/ for more alternatives to robots, and try out the demos

## Compile and run
Have the required depedencies of the frameworks (look at each of their respective repositories), but please note that anything above ODE 1.11.1 is not supported, and will not work due to concurrency issues.

Compile robdyn using `./waf configure` and `./waf` in the robdyn directory, compile sferes2 using `./waf configure --robdyn=/path/to/GaitAdaptation/robdyn/bld/default/src/ --robdyn-osg=/path/to/GaitAdaptation/robdyn/bld/default/src/ --includes=/path/to/GaitAdaptation/robdyn/src/ --cpp11=yes` and then `./waf --exp gatest`. Then compile limbo using `./waf configure --sferes=/path/to/GaitAdaptation/sferes2 --robdyn=/path/to/GaitAdaptation/robdyn/` and .`/waf --exp gaitopt`.

To run the code start evolving the MapElites archive with the executable gatest in `./sferes2/build/default/exp/gatest/`. This is very computationally expensive, so it might need to run anywhere from multiple hours up until several days depending on your hardware.

If you have an archive your can run this in limbo using the gaitopt-executable in `./limbo_old/build/exp/gaitopt/`. Use gaitopt for headless, and gaitopt_graphic if you want to visualize the search. To run a demo try out `./build/exp/gaitopt/gaitopt /path/to/archive_{some_number}.dat 0.4 0.1 20 30` where 0.4 is a parameter for the matern-kernel, 0.1 is the radians of how steep the slope should be, 20 is the amount of obstacles there should be, and 30 is the size of said obstacles.

The result might look something like this:
![pic of limbo](https://i.imgur.com/SokeSElg.png "limbo visualization")
