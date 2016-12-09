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
>- Monstrous combination of JPS and Theta* which still does not have its own name

Build and Launch
================
>To build the project you can use QtCreator or CMake. Both .pro and CMakeLists files are available in the repository. 
>Notice, that project uses C++11 standart. Make sure that your compiler supports it.
>To launch the compiled file and get a result you need an instance - an input XML file. You can find it in Examples folder (somewhen it will appear :)). 

Input and Output files
======================
>Both files are an XML file with a specific structure. 
>Input file should contain:
>- A tag <b>\<map></b>. It describes the search space. First of all it has <b>\<width></b> and <b>\<height></b> tags. 
>- ....

