# RRT-

This repository contains my own python implementations of RRT and RRT*, in both Python and C.

For the C implementation, I am using Visual Studio 2022, and the [CFSML 2.5.1 graphics library](https://www.sfml-dev.org/download/csfml/). To run this code, you must download the library yourself and add the location of the library files to the Visual Studio project's properties (see [this tutorial](https://www.sfml-dev.org/tutorials/2.5/start-vc.php)).

In the future, I plan to implement RRT-FN as well, and create some variations of RRT* with my own optimizations: avoiding nodes that are too close to other nodes early on in the program, in order to persuade the algorithm to explore more broadly initially; and adding manhattan distance as a weighted heuristic when deciding whether to pursue a node.

References: https://pdfs.semanticscholar.org/9c35/2fec7a86c875eec17fc054106414b6914b7d.pdf