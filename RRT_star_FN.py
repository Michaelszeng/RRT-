"""
Helper functions and classes for RRT*FN algorithm.

FN = "Fixed Node": RRT* can have issues with high memory utilization due to
adding so many points. FN limits the number of nodes allowed to a fixed number.
After that maximum is reached, new points are only added after randomly removing
an existing point with no children.
"""