
# maze_search.py
# Implements BFS, DFS, and A* search on a grid maze.

from collections import deque
import heapq
import time

# Simple grid: 0 = free, 1 = obstacle
maze = [
    [0,0,0,0,0],
    [1,1,0,1,0],
    [0,0,0,1,0],
    [0,1,1,0,0],
    [0,0,0,0,0]
]

start = (0,0)
goal = (4,4)

def get_neighbors(pos):
    x,y = pos
    dirs = [(1,0),(-1,0),(0,1),(0,-1)]
    for dx,dy in dirs:
        nx,ny = x+dx, y+dy
        if 0 <= nx < 5 and 0 <= ny < 5 and maze[nx][ny] == 0:
            yield (nx,ny)

def bfs(start, goal):
    t0 = time.time()
    queue = deque([start])
    visited = {start: None}
    while queue:
        cur = queue.popleft()
        if cur == goal:
            break
        for nb in get_neighbors(cur):
            if nb not in visited:
                visited[nb] = cur
                queue.append(nb)
    t1 = time.time()
    return reconstruct_path(visited, goal), t1-t0, len(visited)

def dfs(start, goal):
    t0 = time.time()
    stack = [start]
    visited = {start: None}
    while stack:
        cur = stack.pop()
        if cur == goal:
            break
        for nb in get_neighbors(cur):
            if nb not in visited:
                visited[nb] = cur
                stack.append(nb)
    t1 = time.time()
    return reconstruct_path(visited, goal), t1-t0, len(visited)

def heuristic(a,b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def astar(start, goal):
    t0 = time.time()
    pq = [(0, start)]
    visited = {start: None}
    g = {start: 0}
    while pq:
        _, cur = heapq.heappop(pq)
        if cur == goal:
            break
        for nb in get_neighbors(cur):
            new_cost = g[cur] + 1
            if nb not in g or new_cost < g[nb]:
                g[nb] = new_cost
                visited[nb] = cur
                f = new_cost + heuristic(nb, goal)
                heapq.heappush(pq, (f, nb))
    t1 = time.time()
    return reconstruct_path(visited, goal), t1-t0, len(visited)

def reconstruct_path(visited, goal):
    if goal not in visited:
        return []
    path = []
    cur = goal
    while cur is not None:
        path.append(cur)
        cur = visited[cur]
    return path[::-1]

if __name__ == "__main__":
    for name, func in [("BFS", bfs), ("DFS", dfs), ("A*", astar)]:
        path, t, explored = func(start, goal)
        print(name, "path:", path)
        print("Time:", t, " Explored:", explored)
        print()
