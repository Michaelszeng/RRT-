# RRT-

This repository contains my own implementations of RRT, RRT*, RRT-FN after reading about these path planning algorithms.

I also attempt to add some of my own optimizations: avoiding nodes that are too close to other nodes early on in the program, in order to persuade the algorithm to explore more broadly initially. And adding manhattan distance as a weighted heuristic when deciding whether to pursue a node.

References: https://pdfs.semanticscholar.org/9c35/2fec7a86c875eec17fc054106414b6914b7d.pdf

Next step will be to implement in C++.