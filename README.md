# AStar-JPS-ThetaStar
Basic algorithms for single-shot grid-based path finding.

Description
==========
This project contains implementations of the following algorithms:
>- Best First Search
>- Dijkstra's algorithm
>- A*
>- Theta*
>- Jump Point Search(JPS)

Build and Launch
================
>To build the project you can use QtCreator or CMake. Both .pro and CMakeLists files are available in the repository. 
>Notice, that project uses C++11 standart. Make sure that your compiler supports it.
>To launch the compiled file and get a result you need an instance - an input XML file. You can find it in Examples folder (somewhen it will appear :)). 

Input and Output files
======================
>Both files are an XML file with a specific structure. 
>Input file should contain:
>- Mandatory tag <b>\<map></b>. It describes the search space.
  * **\<height>** and **\<width>** - mandatory tags that define size of the map.
  * **\<startx>** and **\<starty>** - mandatory tags that define coordinates of the start location.
  * **\<finishx>** and **\<finishy>** - mandatory tags that define coordinates of the finish location.
  * **\<grid>** - mandatory tag that describes the map and consists of tags **\<row>**. Each **\<row>** contains a sequence of "0" and "1". "0" means that the cell is traversable, "1" - untraversable. Origin of coordinates is in the upper left corner.
  * **\<cellsize>** - optional tag that defines the size of one cell. It needs for counting scaled length of the path.
  * **\<title>**, **\<URL>**, **\<coordinates>**, etc - some optional information about the map.
>- Mandatory tag <b>\<algorithm></b>. It describes the parameters of the algorithm.
  * **\<searchtype>** - mandatory tag that defines the algorithm. Possible values - "astar", "theta", "jp_search", "bfs", "dijkstra".
  * **\<metrictype>** - defines the type of metric for heuristic function. Possible values - "euclidean", "diagonal", "manhattan", "chebyshev". Default value is "euclidean".
  * **\<hweight>** - defines the weight of heuristic function. Default value is "1".
  * **\<breakingties>** - defines the priority in OPEN list when nodes have the equal F-values. Possible values - "g-min", "g-max". Default value is "g-max".
  * **\<allowdiagonal>** - boolean tag that defines the possibility to make diagonal moves. Ignores this tag if algorithm "theta" is chosen. Default value is "true".
  * **\<cutcorners>** - boolean tag that defines the possibilty to make diagonal moves when one adjacent cell is untraversable. Ignores this tag if diagonal moves are disallowed. Default value is "false".
  * **\<allowsquuze>** - boolean tag that defines the possibility to make diagonal moves when both adjacent cells are untraversable. Ignores this tag if cutting corners is disallowed. Default value is "false".
>- Optional tag <b>\<options></b>. It describes the format of output file.
  * **\<loglevel>** - defines the completness of log-file. Default value is "1". Possible values:
    * "0" or "none" - log-file is not created at all.
    * "0.5" or "tiny" - besides the input file, into the log-file is written **\<summary>** that contains main results, such as path length, number of steps, time, etc.
    * "1" or "short" - into the log-file is also written **\<path>** that looks like **\<grid>** but has a path which is marked by "\*" instead of "0". It also contains **\<hplevel>** and **\<lplevel>** which represent the path in different manner. **\<lplevel>** consits of points, whereas **\<hplevel>** consists of sections.
    * "1.5" or "medium" - additionally contains the final states of OPEN and CLOSE lists.
    * "2" or "full" - into the log-file are written the states of OPEN and CLOSE lists after each step of the algorithm. Be careful, it can make really huge files!
  * **\<logpath>** - defines the directory where the log-file should be written. Default value is the directory of input file.
  * **\<logname>** - defines the name of log-file. Default value is "input file name"+"_log"+"input file extension". 
