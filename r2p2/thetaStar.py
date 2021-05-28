import path_planning as pp
from math import inf

def children(point,grid):
    """
        Calculates the children of a given node over a grid.
        Inputs:
            - point: node for which to calculate children.
            - grid: grid over which to calculate children.
        Outputs:
            - list of children for the given node.
    """
    x,y = point.grid_point
    if x > 0 and x < len(grid) - 1:
        if y > 0 and y < len(grid[0]) - 1:
            links = [grid[d[0]][d[1]] for d in\
                     [(x-1, y),(x,y - 1),(x,y + 1),(x+1,y),\
                      (x-1, y-1), (x-1, y+1), (x+1, y-1),\
                      (x+1, y+1)]]
        elif y > 0:
            links = [grid[d[0]][d[1]] for d in\
                     [(x-1, y),(x,y - 1),(x+1,y),\
                      (x-1, y-1), (x+1, y-1)]]
        else:
            links = [grid[d[0]][d[1]] for d in\
                     [(x-1, y),(x,y + 1),(x+1,y),\
                      (x-1, y+1), (x+1, y+1)]]
    elif x > 0:
        if y > 0 and y < len(grid[0]) - 1:
            links = [grid[d[0]][d[1]] for d in\
                     [(x-1, y),(x,y - 1),(x,y + 1),\
                      (x-1, y-1), (x-1, y+1)]]
        elif y > 0:
            links = [grid[d[0]][d[1]] for d in\
                     [(x-1, y),(x,y - 1),(x-1, y-1)]]
        else:
            links = [grid[d[0]][d[1]] for d in\
                     [(x-1, y), (x,y + 1), (x-1, y+1)]]
    else:
        if y > 0 and y < len(grid[0]) - 1:
            links = [grid[d[0]][d[1]] for d in\
                     [(x+1, y),(x,y - 1),(x,y + 1),\
                      (x+1, y-1), (x+1, y+1)]]
        elif y > 0:
            links = [grid[d[0]][d[1]] for d in\
                     [(x+1, y),(x,y - 1),(x+1, y-1)]]
        else:
            links = [grid[d[0]][d[1]] for d in\
                     [(x+1, y), (x,y + 1), (x+1, y+1)]]
    return [link for link in links if link.value != 9]

def lineofsigth(current, node, grid):
    x0 = current.grid_point[0]
    y0 = current.grid_point[1]
    x1 = node.grid_point[0]
    y1 = node.grid_point[1]
    dy = y1 - y0
    dx = x1 - x0
    f = 0
    sx, sy = 0, 0
    if dy < 0:
        dy = -dy
        sy = -1
    else:
        sy = 1
    if dx < 0:
        dx = -dx
        sx = -1
    else:
        sx = 1
    if dx >= dy:
        while x0 != x1:
            f = f + dy
            if f >= dx:
                if grid[x0 + ((sx - 1) // 2)][y0 + ((sy - 1) // 2)].value >= 5:
                    return False
                y0 = y0 + sy
                f = f - dx
            if f != 0 and grid[x0 + ((sx - 1) // 2)][y0 + ((sy - 1) // 2)].value >= 5:
                return False
            if dy == 0 and grid[x0 + ((sx - 1) // 2)][y0].value >= 5 and grid[x0 + ((sx - 1) // 2)][y0 - 1].value >= 5:
                return False
            x0 = x0 + sx
    else:
        while y0 != y1:
            f = f + dx
            if f >= dy:
                if grid[x0 + ((sx - 1) // 2)][y0 + ((sy - 1) // 2)].value >= 5:
                    return False
                x0 = x0 + sx
                f = f - dy
            if f != 0 and grid[x0 + ((sx - 1) // 2)][y0 + ((sy - 1) // 2)].value >= 5:
                return False
            if dx == 0 and grid[x0][y0 + ((sy - 1) // 2)].value >= 5 and grid[x0 - 1][y0 + ((sy - 1) // 2)].value >= 5:
                return False
            y0 = y0 + sy
    return True

def thetaStar(start, goal, grid, heur='naive'):
    """
        Executes the Tetha* path planning algorithm over a given grid.
        Inputs:
            - start: node from which to start.
            - goal: node to which it is desired to arrive.
            - grid: grid over which to execute the algorithm
            - heur: heuristic function to use for the algorithm,
            expressed as a string. Results will vary depending on
            it. Must be implemented separatedly.
        Outputs:
            - ordered list of nodes representing the shortest path found
            from start to goal.
    """
    #The open and closed sets
    openset = set()
    closedset = set()
    #Current point is the starting point
    current = start
    #Add the starting point to the open set
    openset.add(current)
    #While the open set is not empty
    while openset:
        #Find the item in the open set with the lowest G + H score
        current = min(openset, key=lambda o:o.G + o.H)
        pp.expanded_nodes += 1
        #If it is the item we want, retrace the path and return it
        if current == goal:
            path = []
            while current.parent:
                path.append(current)
                current = current.parent
            path.append(current)
            return path[::-1]
        #Remove the item from the open set
        openset.remove(current)
        #Add it to the closed set
        closedset.add(current)
        #Loop through the node's children/siblings
        for node in children(current,grid):
            #If it is already in the closed set, skip it
            if node in closedset:
                continue
            if node not in openset:
                node.G = inf
                node.parent = None
            if current.parent != None and lineofsigth(current.parent, node, grid):
                new_g = current.parent.G + current.parent.move_cost(node)
                if new_g < node.G:
                    node.G = new_g
                    node.parent = current.parent
                    if node in openset:
                        openset.remove(node)
                    openset.add(node)
            else:
                #Check if we beat the G score
                new_g = current.G + current.move_cost(node)
                if node.G > new_g:
                    #If so, update the node to have a new parent
                    node.G = new_g
                    node.parent = current
                    if node in openset:
                        openset.remove(node)
                    openset.add(node)
    #Throw an exception if there is no path
    raise ValueError('No Path Found')

pp.register_search_method('Theta*', thetaStar)