# AStar-JPS-ThetaStar
Basic algorithms for single-shot grid-based 2D path finding.

Single-shot means that the algorithms are tailored to answering a single pathfinding query (as opposed to other pathfinders that are suited to solving sequences of alike queries, e.g. queries on the same map but with different start and goal locations).

Grid-based means that a (regular square) grid is an essential part of the input. We follow center-based assumption which implies that distinct agent's locations are tied to the centers of the traversable cells (not to the corners). 

Description
==========
This project contains implementations of the following algorithms:
- Breadth First Search
- Dijkstra's algorithm
- A*
- Theta*
- Jump Point Search (JPS)

Build and Launch
================
To build the project you can use QMake (part of QT toolchain) or CMake. Both .pro and CMakeLists files are available in the repo.
Please note that the code relies on C++11 standart. Make sure that your compiler supports it.
The project does not use any external libraries (except tinyXML which is linked at the source level) and is meant to be cross-platform.

To launch the compiled file and get a result you need to pass a correct input XML-file (see below) as the first (command line) argument. This file encodes all the information about path finding instance (map, start, goal, algorithm's options etc.).

If the input is correct you'll see the result in the console. Detailed XML log-file will also be created by default. If you choose not to create output XML you can alter the input XML following the instructions given below.

Input and Output files
======================
Both files are XML files of a specific structure. 
Input file should contain:
>- Mandatory tag <b>\<map></b>. It describes the environment.
  * **\<height>** and **\<width>** - mandatory tags that define size of the map. Origin is in the upper left corner. (0,0) - is upper left, (*width*-1, *height*-1) is lower right.
  * **\<startx>** and **\<starty>** - mandatory tags that define horizontal (X) and vertical (Y) offset of the start location from the upper left corner. Legal values for *startx* are [0, .., *width*-1], for *starty* - [0, .., *height*-1].
  * **\<finishx>** and **\<finishy>** - mandatory tags that horizontal (X) and vertical (Y) offset of the goal location.
  * **\<grid>** - mandatory tag that describes the square grid constituting the map. It consists of **\<row>** tags. Each **\<row>** contains a sequence of "0" and "1" separated by blanks. "0" stands for traversable cell, "1" - for untraversable (actually any other figure but "0" can be used instead of "1").
  * **\<cellsize>** - optional tag that defines the size of one cell. One might add it to calculate scaled length of the path.
  * **\<title>**, **\<URL>**, **\<coordinates>**, etc - optional tags containing additional information on the map.
>- Mandatory tag <b>\<algorithm></b>. It describes the parameters of the algorithm.
  * **\<searchtype>** - mandatory tag that defines which algorithm will be used for pathfinding. Possible values - "astar", "theta", "jp_search", "bfs", "dijkstra".
  * **\<metrictype>** - defines the type of metric for heuristic function. Possible values - "euclid", "diagonal", "manhattan", "chebyshev". Default value is "euclid". Please note that the lenghs of the actual moves (not the heuristic to goal) are always counted with Euclid metrics, e.g. the length of the move between two cardinally adjacent cells is 1, the length of the move between two diagonally adjacent cells is sqrt(2) etc.
  * **\<hweight>** - defines the weight of the heuristic function. Should be real number greater or equal 1. Default value is "1".
  * **\<breakingties>** - defines the priority in OPEN list for nodes with equal f-values. Possible values - "g-min" (break ties in favor of the node with smaller g-value), "g-max" (break ties in favor of the node with greater g-value). Default value is "g-max".
  * **\<allowdiagonal>** - boolean tag that allows/prohibits diagonal moves. Default value is "true". Setting it to "false" restricts the agent to make cardinal (horizonal, vertical) moves only. If Theta* algorithm is used only cardinal successors will be generated during expansion of the current node, but after resetting parent the resultant move will probably violate this restriction. 
  * **\<cutcorners>** - boolean tag that defines the possibilty to make diagonal moves when one adjacent cell is untraversable. The tag is ignored if diagonal moves are not allowed. Default value is "false".
  * **\<allowsqueeze>** - boolean tag that defines the possibility to make diagonal moves when both adjacent cells are untraversable. The tag is ignored if cutting corners is not allowed. Default value is "false".
>- Optional tag <b>\<options></b>. Options that are not related to search.
  * **\<loglevel>** - defines the level of detalization of log-file. Default value is "1". Possible values:
    * "0" or "none" - log-file is not created.
    * "0.5" or "tiny" - All the input data is copied to the log-file plus short **\<summary>** is appended. **\<summary>** contains info of the path length, number of steps, elapsed time, etc.
    * "1" or "short" - *0.5*-log plus **\<path>** is appended. It looks like **\<grid>** but cells forming the path are marked by "\*" instead of "0". The following tags are also appended: **\<hplevel>** and **\<lplevel>**. **\<lplevel>** is the sequence of coordinates of cells forming the path (in case Theta* planner is used, this sequence is formed at post-processing step by invoking sequantually line-of-sight procedure on path's segments). **\<hplevel>** is the sequence of sections forming the path (in case planner other from Theta* is used, sections are formed at post-processing step using naive procedure).
    * "1.5" or "medium" - *1*-log plus the information (explicit enumeration) on last iteration's OPEN and CLOSE lists.
    * "2" or "full" - *1*-log plus OPEN and CLOSE lists are written into the log-file after each step of the algorithm. Can make log-files really huge.
  * **\<logpath>** - defines the directory where the log-file should be written. If not specified directory of the input file is used. 
  * **\<logname>** - defines the name of log-file. If not specified the name of the log file is: "input file name"+"_log"+input file extension.
